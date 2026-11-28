#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::format;
use defmt::println;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::TcpClient;
use embassy_net::{Ipv4Address, Stack, tcp::TcpSocket};
use embassy_net::{Runner, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer, with_timeout};
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::Uart;
use esp_hal::{Async, Blocking};
use esp_hal::{clock::CpuClock, rng::Rng};
use esp_println::print;
use esp_radio::esp_now::{EspNow, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo};
use esp_radio::wifi::scan::ScanConfig;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Password, WifiController,
    sta::StationConfig,
};
use log::info;
use one_prop::cmd::{ControlCommand, decode_command};
use one_prop::controls::Controls;
use one_prop::data::{ControlPacket, PayloadData, decode_packet, encode_packet};
use one_prop::pid::PidConfig;
use smoltcp::iface::Interface;
use static_cell::StaticCell;
use trouble_host::prelude::HeaplessString;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

static CONTROLS: Signal<CriticalSectionRawMutex, Controls> = Signal::new();
static PID_CONFIG: Signal<CriticalSectionRawMutex, PidConfig> = Signal::new();

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

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 32 * 1024);
    // 1. Initialize the Wi-Fi driver backend
    let controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default()).unwrap();

    let mut esp_now = controller.esp_now();
    esp_now.set_channel(1).unwrap();

    //usb0 = 68:fe:71:87:f7:44
    //usb1 = 68:fe:71:12:65:3c

    // this board is 3c - sending to 10
    //let peer_mac = [0x1c, 0x8f, 0x57, 0x2f, 0xcd, 0x10];
    //new       1c:8f:57:2f:cd:10

    let peer_mac = [0x68, 0xfe, 0x71, 0x12, 0x65, 0x3c];
    //let peer_mac = [0x1c, 0x8f, 0x57, 0x2f, 0xcd, 0x10];
    //let peer_mac = [0x20, 0x50, 0x0d, 0x4d, 0x3b, 0x10];

    // this is cd 10

    log::info!("sending to: '0x3b 10'");
    //let peer_mac: [u8; 6] = if PEER.trim() == "A" {
    //    log::info!("This is peer A, sending to (3c)");
    //    [0x68, 0xfe, 0x71, 0x12, 0x65, 0x3c]
    //} else {
    //    log::info!("This is not peer A, sending to (0x44)");
    //    [0x68, 0xfe, 0x71, 0x87, 0xf7, 0x44]
    //};
    esp_now
        .add_peer(PeerInfo {
            interface: EspNowWifiInterface::Station,
            peer_address: peer_mac,
            lmk: None,
            channel: Some(1),
            encrypt: false,
        })
        .unwrap();

    let duration = Duration::from_secs(1);

    let (manager, sender, receiver) = esp_now.split();

    spawner.spawn(read_uart(uart).unwrap());
    spawner.spawn(rec_telemetery(receiver).unwrap());
    spawner.spawn(send_controls(sender).unwrap());

    //let mut seq: u8 = 0;
    //let mut ticker = Ticker::every(Duration::from_millis(50)); // 20 Hz

    //let mut controls = Controls {
    //    x: 0.,
    //    y: 0.,
    //    yaw: 0.,
    //    throttle: 0.,
    //};

    //loop {
    //    // GET CONTROLS FROM UART
    //    if let Some(ctrls) = CONTROLS.try_take() {
    //        controls = ctrls
    //    };
    //    // SEND CONTROLS
    //    match esp_now
    //        .send_async(
    //            &peer_mac,
    //            &encode_packet(PayloadData::Control(ControlPacket {
    //                seq,
    //                x: controls.x,
    //                y: controls.y,
    //                throttle: controls.throttle,
    //                yaw: controls.yaw,
    //            })),
    //        )
    //        .await
    //    {
    //        Ok(_) => {
    //            //log::info!("Sent!")
    //        }
    //        Err(err) => log::warn!("Send err: {:?}", err),
    //    };

    //    // TELEMENTARY
    //    //match with_timeout(duration, esp_now.receive_async()).await {
    //    //    Ok(res) => {
    //    //        let data = res.data();
    //    //        let p = decode_packet(data).unwrap();
    //    //    }
    //    //    Err(err) => {
    //    //        log::warn!("Receive timeout: {:?}", err);
    //    //    }
    //    //}

    //    seq = seq.wrapping_add(1);
    //    ticker.next();
    //}

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

#[embassy_executor::task]
pub async fn send_pid_config() {}

#[embassy_executor::task]
pub async fn rec_telemetery(mut esp_now: EspNowReceiver) {
    let duration = Duration::from_millis(1000);
    let mut ticker = Ticker::every(Duration::from_hz(20));

    loop {
        // TELEMENTARY
        match with_timeout(duration, esp_now.receive_async()).await {
            Ok(res) => {
                let data = res.data();
                let p = decode_packet(data).unwrap();
                log::info!("rec: {:?}", p);
            }
            Err(err) => {
                log::warn!("Receive timeout: {:?}", err);
            }
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn send_controls(mut esp_now: EspNowSender) {
    let peer_mac = [0x68, 0xfe, 0x71, 0x12, 0x65, 0x3c];
    //let peer_mac = [0x1c, 0x8f, 0x57, 0x2f, 0xcd, 0x10];
    //let peer_mac = [0x20, 0x50, 0x0d, 0x4d, 0x3b, 0x10];

    let mut ticker = Ticker::every(Duration::from_hz(50));
    let mut seq = 0;

    let mut controls = Controls {
        x: 0.,
        y: 0.,
        yaw: 0.,
        throttle: 0.,
    };
    loop {
        // GET CONTROLS FROM UART
        if let Some(ctrls) = CONTROLS.try_take() {
            controls = ctrls
        };
        // SEND CONTROLS
        match esp_now
            .send_async(
                &peer_mac,
                &encode_packet(PayloadData::Control(ControlPacket {
                    seq,
                    x: controls.x,
                    y: controls.y,
                    throttle: controls.throttle,
                    yaw: controls.yaw,
                })),
            )
            .await
        {
            Ok(_) => {
                //log::info!("Sent!")
            }
            Err(err) => log::warn!("Send err: {:?}", err),
        };

        if let Some(config) = PID_CONFIG.try_take() {
            match esp_now
                .send_async(&peer_mac, &encode_packet(PayloadData::PidConfig(config)))
                .await
            {
                Ok(_) => {
                    //log::info!("Sent pid!")
                }
                Err(err) => log::warn!("Send err: {:?}", err),
            };
        };

        seq = seq.wrapping_add(1);
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn read_uart(mut uart: Uart<'static, Async>) {
    let mut buf = [0u8; 32];
    let mut y = 0.;
    let mut x = 0.;
    let mut throttle = 0.;
    let mut pid_config = PidConfig::default();
    let mut line = HeaplessString::<64>::new();

    let mut budget = 128usize;
    loop {
        match uart.read_async(&mut buf).await {
            Ok(n) => {
                let mut pid_changed = false;
                let mut ctrls_changed = false;
                budget = budget.saturating_sub(n.max(1));
                for &b in &buf[..n] {
                    if b == b'\n' || b == b'\r' {
                        if !line.is_empty() {
                            if let Some(command) = decode_command(line.as_bytes()) {
                                match command {
                                    ControlCommand::Bank(v) => {
                                        x = v;
                                        ctrls_changed = true
                                    }
                                    ControlCommand::Pitch(v) => {
                                        y = v;
                                        ctrls_changed = true
                                    }
                                    ControlCommand::Throttle(v) => {
                                        throttle = v;
                                        ctrls_changed = true
                                    }
                                    ControlCommand::Pid(kp, ki, kd) => {
                                        log::info!("received pid command from uart");
                                        // TODO: only updating gains
                                        pid_config = PidConfig {
                                            kp,
                                            ki,
                                            kd,
                                            integral_limit: 0.,
                                            output_limit: 1.,
                                            d_alpha: 0.2,
                                        };
                                        pid_changed = true;
                                    }
                                    _ => {}
                                }
                            }
                            line.clear();
                        }
                    } else if b.is_ascii() && line.push(b as char).is_err() {
                        line.clear(); // line too long, drop it
                    }
                }
                if ctrls_changed {
                    CONTROLS.signal(Controls {
                        x,
                        y,
                        throttle,
                        yaw: 0.,
                    });
                };
                if pid_changed {
                    PID_CONFIG.signal(pid_config);
                };
            }
            Err(err) => {
                log::warn!("uart error: {}", err);
                line.clear();
            }
        }
    }
}
