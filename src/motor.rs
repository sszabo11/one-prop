use esp_hal::ledc::{LowSpeed, channel::ChannelIFace};

pub fn set_esc_throttle<'a>(channel: &mut impl ChannelIFace<'a, LowSpeed>, throttle_0_to_100: u16) {
    let max_duty = 16_383u32; // 14-bit
    let period_us = 20_000u32; // 50 Hz

    // clamp 0..=100
    let t = (throttle_0_to_100 as u32).clamp(0, 100);

    // map 0..100 → 1000..2000 µs
    let pulse_us = 1000 + (t * 1000) / 100; // 1000..2000
    // same as: 1000 + t * 10

    let duty = (pulse_us * max_duty) / period_us;

    // IMPORTANT: do not cast duty to u8
    channel.set_duty(duty as u8).ok();
    // if your API is set_duty_cycle_fraction / different width, match that type
}

pub fn set_esc_throttle2<'a>(
    channel: &mut impl ChannelIFace<'a, LowSpeed>,
    throttle_0_to_100: u16,
) {
    let max_duty = 255u32; // 8-bit
    let period_us = 20_000u32; // 50 Hz

    let t = (throttle_0_to_100 as u32).clamp(0, 100);

    // 0..100 → 1000..2000 µs
    let pulse_us = 1000 + t * 10;

    // pulse → duty
    let duty = (pulse_us * max_duty) / period_us; // ~12..25
    let duty = duty.clamp(0, 255) as u8;

    channel.set_duty(duty);
}
