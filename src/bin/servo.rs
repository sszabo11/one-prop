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
use core::f32::consts::TAU;
use defmt::println;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::TcpClient;
use embassy_net::{Ipv4Address, Stack, tcp::TcpSocket};
use embassy_net::{Runner, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as ChannelSync;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker, Timer, with_timeout};
use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use embedded_io_async::Write;
use esp_hal::analog::adc::{Adc, AdcConfig, AdcPin, Attenuation};
use esp_hal::delay::Delay;
use esp_hal::i2c::master::I2c;
use esp_hal::ledc::channel::{Channel, ChannelHW, ChannelIFace};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{self, HighSpeed, LSGlobalClkSource, Ledc, LowSpeed, channel, timer};
use libm::{cosf, sinf}; // no_std has no f32 sin/cos in core; micromath also works
use nb;

use esp_backtrace as _;
use esp_hal::gpio::{AnalogPin, DriveMode, InputPin};
use esp_hal::peripherals::{ADC1, GPIO33, GPIO34, GPIO35, WIFI};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;

use esp_hal::uart::Uart;
use esp_hal::{Async, Blocking, DriverMode, i2c};
use esp_hal::{clock::CpuClock, rng::Rng};
use esp_println::print;
use esp_radio::esp_now::{EspNow, EspNowWifiInterface, PeerInfo};
use esp_radio::wifi::scan::ScanConfig;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Password, WifiController,
    sta::StationConfig,
};
use log::info;
use mpu6050_dmp::sensor::Mpu6050;
use num_traits::float::FloatCore;
use one_prop::cmd::{ControlCommand, decode_command};
use one_prop::controls::Controls;
use one_prop::data::{ControlPacket, PayloadData, TelemetryPacket, decode_packet, encode_packet};
use one_prop::inputs::{Actuators, setup_actuators};
use one_prop::motor::{set_esc_throttle, set_esc_throttle2};
use smoltcp::iface::Interface;
use static_cell::StaticCell;
use trouble_host::prelude::HeaplessString;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

static CONTROLS: Signal<CriticalSectionRawMutex, Controls> = Signal::new();
static HSTIMER0: StaticCell<ledc::timer::Timer<'static, HighSpeed>> = StaticCell::new();
static LSTIMER0: StaticCell<ledc::timer::Timer<'static, LowSpeed>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

type ThrottlePin = AdcPin<GPIO34<'static>, ADC1<'static>>;
type VrxPin = AdcPin<GPIO35<'static>, ADC1<'static>>;
type VryPin = AdcPin<GPIO33<'static>, ADC1<'static>>;

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

    let mut adc_config = AdcConfig::new();
    let pot_pin = peripherals.GPIO34;
    let mut pot_pin = adc_config.enable_pin(pot_pin, Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc_config);

    let mut actuators = setup_actuators(
        ledc,
        peripherals.GPIO13, // x
        peripherals.GPIO14, // y
        peripherals.GPIO15, // Elevator
        peripherals.GPIO32, // Throttle
    );

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 32 * 1024);

    let mut latest = Controls::default();
    let mut ticker = Ticker::every(Duration::from_millis(20));
    let max_duty_cycle = actuators.x.max_duty_cycle() as u32;

    let min_duty = (25 * max_duty_cycle) / 1000;
    let max_duty = (125 * max_duty_cycle) / 1000;
    let duty_gap = max_duty - min_duty;

    let max_duty = 16383u32;

    let period_us = 20_000u32;
    let us_to_duty = |us: u32| -> u32 { (us * max_duty) / period_us };

    let mut bank = -40.;
    let mut forw_x = true;
    let mut forw_y = true;

    let mut x = -30.;
    let mut y = -30.;
    let radius = 25.0_f32; // tilt in degrees, keep within your ±30° limit
    let mut phase = 0.0_f32;
    let phase_step = TAU / 200.0; // 200 ticks per revolution

    let period_us = 20_000u32;
    let us_to_duty = |us: u32| -> u32 { (us * max_duty) / period_us };

    loop {
        let x = radius * cosf(phase);
        let y = radius * sinf(phase);

        let _ = actuators
            .x
            .set_duty_cycle(duty_from_bank_angle(x, min_duty, duty_gap));
        let _ = actuators
            .y
            .set_duty_cycle(duty_from_bank_angle(y, min_duty, duty_gap));

        let raw: u16 = nb::block!(adc1.read_oneshot(&mut pot_pin)).unwrap();

        let pulse_us: u32 = 1000 + (raw as u32 * 1000) / 4095;
        let pulse_us: u32 = pulse_us.clamp(1000, 2000);
        log::info!("adc: {}", raw);
        let duty = us_to_duty(pulse_us);
        log::info!("pulse_us={}  duty={}", pulse_us, duty);

        actuators.throttle.set_duty_hw(duty);

        phase += phase_step;
        if phase >= TAU {
            phase -= TAU;
        }

        ticker.next().await;
    }
}

pub fn duty_from_bank_angle(deg: f32, min_duty: u32, duty_gap: u32) -> u16 {
    let deg = deg.clamp(-90.0, 90.0);
    let t = (deg + 90.0) / 180.0; // 0.0 … 1.0
    let duty = min_duty as f32 + t * duty_gap as f32;
    duty.round() as u16
}
