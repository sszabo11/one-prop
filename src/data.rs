use esp_println::println;
use serde::{Deserialize, Serialize};

use crate::pid::PidConfig;

#[derive(Serialize, Deserialize, Debug)]
pub enum PayloadData {
    Telemetry(TelemetryPacket),
    Control(ControlPacket),
    PidConfig(PidConfig),
    None,
    Invalid,
    Timeout,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TelemetryPacket {
    pub roll: f32,
    pub pitch: f32,
    pub seq: u8,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ControlPacket {
    pub seq: u8,
    pub x: f32,
    pub y: f32,
    pub throttle: f32,
    pub yaw: f32,
}

/*
 * [0] = kind/id (0xC1)
 * [2]
 * */
pub fn encode_packet(p: PayloadData) -> [u8; 8] {
    match p {
        PayloadData::Control(data) => {
            let mut p = [0u8; 8];

            p[0] = 0xC1;
            p[1] = data.seq;
            // Control packet
            let x = p[2..4].copy_from_slice(&((data.x * 10.) as i16).to_le_bytes());
            let y = p[4..6].copy_from_slice(&((data.y * 10.) as i16).to_le_bytes());
            let throttle = p[6..7].copy_from_slice(&((data.throttle) as u8).to_le_bytes());
            let yaw = p[7..8].copy_from_slice(&((data.yaw) as u8).to_le_bytes());
            p
        }
        PayloadData::Telemetry(data) => {
            // Telemetry packet
            let mut p = [0u8; 8];
            p[0] = 0xC2;
            p[1] = data.seq;
            let roll = p[2..4].copy_from_slice(&((data.roll * 1000.) as i16).to_le_bytes());
            let pitch = p[4..6].copy_from_slice(&((data.pitch * 1000.) as i16).to_le_bytes());
            p
        }
        PayloadData::PidConfig(data) => {
            let mut p = [0u8; 8];

            p[0] = 0xC3;

            let kp = p[1..3].copy_from_slice(&((data.kp * 1000.) as i16).to_le_bytes());
            let ki = p[3..5].copy_from_slice(&((data.ki * 1000.) as i16).to_le_bytes());
            let kd = p[5..7].copy_from_slice(&((data.kd * 1000.) as i16).to_le_bytes());
            p
        }
        _ => {
            let mut p = [0u8; 8];
            p[0] = 0xC4;
            p
        }
    }
}

pub fn decode_packet(p: &[u8]) -> Result<PayloadData, PayloadData> {
    if p.len() < 8 && !(p[0] == 0xC1 || p[0] == 0xC2 || p[0] == 0xC3 || p[0] == 0xC4) {
        return Ok(PayloadData::None);
    }

    let kind = p[0]; // 0xC1, 0xC2, 0xC3
    //println!("kind: {}", kind);

    match kind {
        0xC1 => {
            // Control packet
            let seq = p[1];
            let x = i16::from_le_bytes([p[2], p[3]]);
            let y = i16::from_le_bytes([p[4], p[5]]);
            let throttle = u8::from_le_bytes([p[6]]);
            let yaw = u8::from_le_bytes([p[7]]);
            Ok(PayloadData::Control(ControlPacket {
                seq,
                x: x as f32 / 10.,
                y: y as f32 / 10.,
                throttle: throttle as f32,
                yaw: yaw as f32,
            }))
        }
        0xC2 => {
            // Telemetry packet
            let seq = p[1];
            let roll = i16::from_le_bytes([p[2], p[3]]);
            let pitch = i16::from_le_bytes([p[4], p[5]]);

            Ok(PayloadData::Telemetry(TelemetryPacket {
                roll: roll as f32 / 1000.,
                pitch: pitch as f32 / 1000.,
                seq,
            }))
        }
        0xC3 => {
            // Pid packet
            let kp = i16::from_le_bytes([p[1], p[2]]);
            let ki = i16::from_le_bytes([p[3], p[4]]);
            let kd = i16::from_le_bytes([p[5], p[6]]);

            Ok(PayloadData::PidConfig(PidConfig {
                kp: kp as f32 / 1000.,
                ki: ki as f32 / 1000.,
                kd: kd as f32 / 1000.,

                // these arent being counted when sent
                integral_limit: 0.,
                output_limit: 0.,
                d_alpha: 0.,
            }))
        }
        _ => Err(PayloadData::Invalid),
    }
}
