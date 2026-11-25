use esp_hal::{
    gpio::{Output, OutputPin, Pin},
    ledc::{
        self, LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, Channel, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::Peripherals,
    time::Rate,
};
use static_cell::StaticCell;

pub struct Actuators {
    pub x: Channel<'static, LowSpeed>,
    pub y: Channel<'static, LowSpeed>,
    pub elevator: Channel<'static, LowSpeed>,
    pub throttle: Channel<'static, LowSpeed>,
}

static LSTIMER0: StaticCell<ledc::timer::Timer<'static, LowSpeed>> = StaticCell::new();

pub fn setup_actuators(
    ledc: Ledc<'static>,
    pin_x: impl OutputPin + 'static,
    pin_y: impl OutputPin + 'static,
    pin_ele: impl OutputPin + 'static,
    pin_thr: impl OutputPin + 'static,
) -> Actuators {
    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer1);

    let mut channel0 = ledc.channel(channel::Number::Channel0, pin_x);
    let mut channel1 = ledc.channel(channel::Number::Channel1, pin_y);
    let mut channel2 = ledc.channel(channel::Number::Channel2, pin_ele);
    let mut channel3 = ledc.channel(channel::Number::Channel3, pin_thr);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    let lstimer0: &'static ledc::timer::Timer<'static, LowSpeed> = LSTIMER0.init(lstimer0);
    channel0
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();
    channel1
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();
    channel2
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();
    channel3
        .configure(channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();

    Actuators {
        x: channel0,
        y: channel1,
        elevator: channel2,
        throttle: channel3,
    }
}
