#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::format;
use bt_hci::{controller::ExternalController, uuid::appearance::computer::PALM_SIZE_PCPDA};
use defmt::println;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_sync::channel::Channel as ChannelSync;
use embassy_sync::watch::Watch;
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

use embassy_net::tcp::client::TcpClient;
use embassy_net::{Ipv4Address, Stack, tcp::TcpSocket};
use embassy_net::{Runner, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use embedded_io_async::Write;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::I2c;
use esp_hal::ledc::channel::{Channel, ChannelHW, ChannelIFace};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, HighSpeed, LSGlobalClkSource, Ledc, LowSpeed, channel, timer};

use esp_backtrace as _;
use esp_hal::gpio::DriveMode;
use esp_hal::peripherals::WIFI;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;

use esp_hal::uart::Uart;
use esp_hal::{Async, Blocking, DriverMode, i2c};
use esp_hal::{clock::CpuClock, rng::Rng};
use esp_println::print;
use esp_radio::esp_now::{EspNow, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo};
use esp_radio::wifi::scan::ScanConfig;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Password, WifiController,
    sta::StationConfig,
};
use log::info;
use mpu6050_dmp::quaternion::Quaternion;
use mpu6050_dmp::sensor::Mpu6050;
use mpu6050_dmp::yaw_pitch_roll::YawPitchRoll;
use num_traits::float::FloatCore;
use one_prop::cmd::{ControlCommand, decode_command};
use one_prop::controls::Controls;
use one_prop::data::PayloadData::Telemetry;
use one_prop::data::{ControlPacket, PayloadData, TelemetryPacket, decode_packet, encode_packet};
use one_prop::inputs::{Actuators, setup_actuators};
use one_prop::motor::{set_esc_throttle, set_esc_throttle2};
use one_prop::pid::{Pid, PidConfig};
use one_prop::telemetery::Telemetery;
use smoltcp::iface::Interface;
use static_cell::StaticCell;
use trouble_host::prelude::HeaplessString;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

static CONTROLS: Signal<CriticalSectionRawMutex, Controls> = Signal::new();
static PID_CONFIG: Signal<CriticalSectionRawMutex, PidConfig> = Signal::new();
static TELEMETERY: Watch<CriticalSectionRawMutex, Telemetery, 2> = Watch::new();
static HSTIMER0: StaticCell<ledc::timer::Timer<'static, HighSpeed>> = StaticCell::new();
static LSTIMER0: StaticCell<ledc::timer::Timer<'static, LowSpeed>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 2.0.0
    // generator parameters: -o esp32 -o esp32-wroom-32d -o unstable-hal -o alloc -o embassy -o wifi -o ble-trouble -o log -o esp-backtrace -o neovim-classic -o esp

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let mut uart = Uart::new(
        peripherals.UART0,
        esp_hal::uart::Config::default().with_baudrate(921_600),
    )
    .unwrap()
    .with_rx(peripherals.GPIO3)
    .with_tx(peripherals.GPIO1)
    .into_async();

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO0
    // - GPIO2
    // - GPIO5
    // - GPIO12
    // - GPIO15
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _gpio6 = peripherals.GPIO6;
    let _gpio7 = peripherals.GPIO7;
    let _gpio8 = peripherals.GPIO8;
    let _gpio9 = peripherals.GPIO9;
    let _gpio10 = peripherals.GPIO10;
    let _gpio11 = peripherals.GPIO11;
    let _gpio16 = peripherals.GPIO16;
    let _gpio20 = peripherals.GPIO20;

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut actuators = setup_actuators(
        ledc,
        peripherals.GPIO13, // x
        peripherals.GPIO14, // y
        peripherals.GPIO15, // Elevator
        peripherals.GPIO32, // Throttle
    );
    log::info!("Actuaters Set!");

    //let mut hstimer0: ledc::timer::Timer<'static, HighSpeed> =
    //    ledc.timer::<'static, HighSpeed>(ledc::timer::Number::Timer0);
    //hstimer0
    //    .configure(ledc::timer::config::Config {
    //        duty: ledc::timer::config::Duty::Duty12Bit,
    //        clock_source: ledc::timer::HSClockSource::APBClk,
    //        frequency: Rate::from_hz(50),
    //    })
    //    .unwrap();
    //let hstimer0: &'static ledc::timer::Timer<'static, HighSpeed> = HSTIMER0.init(hstimer0);

    //let mut channel0: Channel<'static, HighSpeed> =
    //    ledc.channel(ledc::channel::Number::Channel0, peripherals.GPIO13);
    //channel0
    //    .configure(ledc::channel::config::Config {
    //        timer: hstimer0,
    //        duty_pct: 10,
    //        drive_mode: DriveMode::PushPull,
    //    })
    //    .unwrap();

    //// servo
    //let mut motor = peripherals.GPIO14;
    //let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer1);

    //lstimer0
    //    .configure(timer::config::Config {
    //        duty: timer::config::Duty::Duty14Bit,
    //        clock_source: timer::LSClockSource::APBClk,
    //        frequency: Rate::from_hz(50),
    //    })
    //    .unwrap();

    //let mut channel1 = ledc.channel(channel::Number::Channel1, servo1);
    //let mut channel2 = ledc.channel(channel::Number::Channel2, servo2);
    //let mut channel3 = ledc.channel(channel::Number::Channel3, servo3);
    //let mut channel4 = ledc.channel(channel::Number::Channel3, motor);

    //let lstimer0: &'static ledc::timer::Timer<'static, LowSpeed> = LSTIMER0.init(lstimer0);
    //channel1
    //    .configure(channel::config::Config {
    //        timer: lstimer0,
    //        duty_pct: 0,
    //        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    //    })
    //    .unwrap();

    //// ESC
    //let mut motor = peripherals.GPIO26;
    //let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer1);

    //lstimer0
    //    .configure(timer::config::Config {
    //        duty: timer::config::Duty::Duty14Bit, // Higher resolution is better
    //        clock_source: timer::LSClockSource::APBClk,
    //        frequency: Rate::from_hz(50), // 50 Hz for ESC
    //    })
    //    .unwrap();

    //let mut channel1 = ledc.channel(channel::Number::Channel1, motor);
    //let lstimer0: &'static ledc::timer::Timer<'static, LowSpeed> = LSTIMER0.init(lstimer0);
    //channel1
    //    .configure(channel::config::Config {
    //        timer: lstimer0,
    //        duty_pct: 0,
    //        drive_mode: esp_hal::gpio::DriveMode::PushPull,
    //    })
    //    .unwrap();

    // ESC
    //arm_motor(&mut actuators.throttle);
    log::info!("Motor armed!");

    let i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22)
        .into_async();

    let mut delay = Delay::new();
    let mut mpu = Mpu6050::new(i2c, mpu6050_dmp::address::Address::default()).unwrap();

    if let Err(e) = mpu.initialize_dmp(&mut delay) {
        log::info!("Failed to init MPU6050: {:?}", e);
    }

    log::info!("MPU6050 Initialised Successfully!");

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 32 * 1024);

    let controller = esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())
        .expect("Failed to create wifi controller");
    let mut esp_now = controller.esp_now();
    esp_now
        .set_channel(1)
        .expect("Failed to set esp now channel");

    //let peer_mac = [0x68, 0xfe, 0x71, 0x12, 0x65, 0x3c];
    //let peer_mac = [0x20, 0x50, 0x0d, 0x4d, 0x3b, 0x10];
    let peer_mac = [0x1c, 0x8f, 0x57, 0x2f, 0xcd, 0x10];
    // this is 3b 10

    log::info!("sending to : '0x3c");

    esp_now
        .add_peer(PeerInfo {
            interface: EspNowWifiInterface::Station,
            peer_address: peer_mac,
            lmk: None,
            channel: Some(1),
            encrypt: false,
        })
        .unwrap();

    let (manager, sender, receiver) = esp_now.split();

    log::info!("ESP-NOW ready, waiting for packets...");

    spawner.spawn(receive_cmds(receiver).unwrap());

    let mut pid_config = PidConfig {
        kp: 0.2,
        ki: 0.0,
        kd: 0.0,
        integral_limit: 0.2,
        output_limit: 1.,
        d_alpha: 0.2,
    };

    let deadline = Instant::now() + Duration::from_secs(3);

    while Instant::now() < deadline {
        log::info!("Waiting for PID config...");

        if let Some(config) = PID_CONFIG.try_take() {
            log::info!("Reading Pid config from signal");
            pid_config = config
        }
        Timer::after(Duration::from_millis(10)).await;
    }
    log::info!(
        "Loaded pid config: kp: {} | ki: {} | kd: {}",
        pid_config.kp,
        pid_config.ki,
        pid_config.kd,
    );

    spawner.spawn(control_task(actuators, pid_config).unwrap());
    spawner.spawn(transmit_tlm(sender).unwrap());
    spawner.spawn(read_imu(mpu).unwrap());

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

#[embassy_executor::task]
async fn receive_cmds(mut esp_now: EspNowReceiver) {
    let mut seq: u8 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(50));

    loop {
        match with_timeout(Duration::from_secs(1), esp_now.receive_async()).await {
            Ok(res) => {
                let data = res.data();
                let p = decode_packet(data).unwrap();

                log::info!("Received to PEER: {:?}", p);

                match p {
                    PayloadData::Control(c) => {
                        CONTROLS.signal(Controls {
                            x: c.x / 2.,
                            y: c.y / 2.,
                            throttle: c.throttle,
                            yaw: c.yaw,
                        });
                    }
                    PayloadData::PidConfig(config) => {
                        PID_CONFIG.signal(config);
                    }
                    _ => {}
                };
            }
            Err(err) => {
                log::warn!("Receive timeout: {:?}", err);
            }
        }

        seq = seq.wrapping_add(1);
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn read_imu(mut mpu: Mpu6050<I2c<'static, Async>>) {
    log::info!("MPU6050 reading...");
    let mut ticker = Ticker::every(Duration::from_hz(200));
    loop {
        let len = mpu.get_fifo_count().unwrap();
        let mut buf = [0u8; 28];
        if len >= 28 {
            let buf = mpu.read_fifo(&mut buf).unwrap();
            let q = Quaternion::from_bytes(&buf[..16]).unwrap().normalize();
            let ypr = YawPitchRoll::from(q);
            //log::info!("{:?}", ypr);
            //let accel = mpu.accel().expect("Failed to read accel");

            //log::info!("x: {} | y: {} | z: {}", accel.x(), accel.y(), accel.z());
            let sender = TELEMETERY.sender();

            let p = Telemetery {
                pitch: ypr.pitch,
                roll: ypr.roll,
            };

            //log::info!("{},{}", p.roll, p.pitch);
            sender.send(p);
        }

        ticker.next().await
    }
}

#[embassy_executor::task]
async fn transmit_tlm(mut esp_now: EspNowSender) {
    let mut seq: u8 = 0;

    //let peer_mac = [0x68, 0xfe, 0x71, 0x12, 0x65, 0x3c];
    //let peer_mac = [0x20, 0x50, 0x0d, 0x4d, 0x3b, 0x10];
    let peer_mac = [0x1c, 0x8f, 0x57, 0x2f, 0xcd, 0x10];
    // this is 65 3c
    let mut ticker = Ticker::every(Duration::from_millis(50));

    let mut telemetry = Telemetery::default();

    let mut rec = TELEMETERY
        .receiver()
        .expect("Failed to get telemetry receiver");

    loop {
        if let Some(tlm) = rec.try_get() {
            telemetry = tlm
        };
        match esp_now
            .send_async(
                &peer_mac,
                &encode_packet(PayloadData::Telemetry(TelemetryPacket {
                    seq,
                    roll: telemetry.roll,
                    pitch: telemetry.pitch,
                })),
            )
            .await
        {
            Ok(_) => {
                log::info!("Sent!")
            }
            Err(err) => log::warn!("Send err: {:?}", err),
        };

        seq = seq.wrapping_add(1);
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn control_task(mut actuators: Actuators, pid_config: PidConfig) {
    let mut latest = Controls::default();
    let mut ticker = Ticker::every(Duration::from_hz(50));
    let max_duty_cycle = actuators.x.max_duty_cycle() as u32;

    let min_duty = (25 * max_duty_cycle) / 1000;
    let max_duty = (125 * max_duty_cycle) / 1000;
    let duty_gap = max_duty - min_duty;

    let max_duty = 16383u32;

    let period_us = 20_000u32;
    let us_to_duty = |us: u32| -> u32 { (us * max_duty) / period_us };

    let mut rec = TELEMETERY
        .receiver()
        .expect("Failed to get telemetry receiver");

    let mut pid_x = Pid::new(pid_config);
    let mut pid_y = Pid::new(pid_config);
    //let mut pid_x = Pid::new(PidConfig {
    //    kp: 0.2,
    //    ki: 0.0,
    //    kd: 0.0,
    //    integral_limit: 0.2,
    //    output_limit: 1.,
    //    d_alpha: 0.2,
    //});
    //let mut pid_y = Pid::new(PidConfig {
    //    kp: 0.2,
    //    ki: 0.0,
    //    kd: 0.0,
    //    integral_limit: 0.2,
    //    output_limit: 1.,
    //    d_alpha: 0.2,
    //});

    let mut imu = Telemetery::default();
    loop {
        if let Some(c) = CONTROLS.try_take() {
            latest = c;
        }
        if let Some(t) = rec.try_get() {
            imu = t
        }

        let pred_x = pid_x.predict(latest.x / 10., imu.roll);
        let pred_y = pid_y.predict(latest.y / 10., imu.pitch);
        log::info!("roll: {} | pitch: {}", imu.roll, imu.pitch);

        let angle_x = 30. * pred_x;
        let angle_y = 30. * pred_y;

        let duty_x = duty_from_bank_angle(angle_x, min_duty, duty_gap);
        let _ = actuators.x.set_duty_cycle(duty_x);

        let duty_y = duty_from_bank_angle(angle_y, min_duty, duty_gap);
        let _ = actuators.y.set_duty_cycle(duty_y);

        if latest.throttle < 4. {
            latest.throttle = 0.;
            pid_x.reset();
            pid_y.reset();
        }
        let raw = (latest.throttle / 100.) * 4095.;
        let pulse_us: u32 = 1000 + (raw as u32 * 1000) / 4095;
        let pulse_us: u32 = pulse_us.clamp(1000, 2000);

        actuators.throttle.set_duty_hw(us_to_duty(pulse_us));

        ticker.next().await;
    }
}
#[embassy_executor::task]
async fn acutator_task(mut actuators: Actuators) {
    let mut latest = Controls::default();
    let mut ticker = Ticker::every(Duration::from_millis(20));
    let max_duty_cycle = actuators.x.max_duty_cycle() as u32;

    let min_duty = (25 * max_duty_cycle) / 1000;
    let max_duty = (125 * max_duty_cycle) / 1000;
    let duty_gap = max_duty - min_duty;

    let max_duty = 16383u32;

    let period_us = 20_000u32;
    let us_to_duty = |us: u32| -> u32 { (us * max_duty) / period_us };
    loop {
        if let Some(c) = CONTROLS.try_take() {
            latest = c;
            let duty_x = duty_from_bank_angle(latest.x, min_duty, duty_gap);
            let _ = actuators.x.set_duty_cycle(duty_x);

            let duty_y = duty_from_bank_angle(latest.y, min_duty, duty_gap);
            let _ = actuators.y.set_duty_cycle(duty_y);
        }

        if latest.throttle < 4. {
            latest.throttle = 0.;
        }
        let raw = (latest.throttle / 100.) * 4095.;
        let pulse_us: u32 = 1000 + (raw as u32 * 1000) / 4095;
        let pulse_us: u32 = pulse_us.clamp(1000, 2000);

        //log::info!("pulse us: {}", pulse_us);
        //log::info!("duty: {}", us_to_duty(pulse_us));
        actuators.throttle.set_duty_hw(us_to_duty(pulse_us));

        ticker.next().await;
    }
}

pub fn duty_from_bank_angle(deg: f32, min_duty: u32, duty_gap: u32) -> u16 {
    let deg = deg.clamp(-90.0, 90.0);
    let t = (deg + 90.0) / 180.0; // 0.0 … 1.0
    let duty = min_duty as f32 + t * duty_gap as f32;
    duty.round() as u16
}

fn arm_motor2(motor: &mut Channel<'static, LowSpeed>) {
    let max_duty = 16383u32;
    let period_us = 20_000u32;
    let us_to_duty = |us: u32| -> u32 {
        // duty = us / period * max_duty
        (us * max_duty) / period_us
    };
    let delay = Delay::new();
    set_esc_throttle(motor, 1000); // 1000 µs = stop
    delay.delay_millis(3000);
    motor.set_duty_hw(us_to_duty(1000));
    delay.delay_millis(4000); // wait for arm

    for _ in 0..150 {
        set_esc_throttle2(motor, 0);
        delay.delay_millis(20);
    }
}
fn arm_motor(motor: &mut Channel<'static, LowSpeed>) {
    let max_duty = 16383u32;
    let period_us = 20_000u32;
    let us_to_duty = |us: u32| (us * max_duty) / period_us;
    let delay = Delay::new();

    // hold minimum throttle (1000 µs) so the ESC arms
    motor.set_duty_hw(us_to_duty(1000));
    delay.delay_millis(5000);
}
