use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    pub integral_limit: f32, // max magnitude of the I contribution (output units)
    pub output_limit: f32,   // max magnitude of the total output
    pub d_alpha: f32,        // D low-pass: 0..1, 1 = no filtering
}

pub struct Pid {
    pub config: PidConfig,
    integral: f32,
    last_meas: f32,
    d_filt: f32,
    primed: bool,
}

const DT: f32 = 1.0 / 50.0;

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 0.05,
            ki: 0.0,
            kd: 0.0,
            integral_limit: 0.2,
            output_limit: 1.,
            d_alpha: 0.3,
        }
    }
}

impl Pid {
    pub fn new(config: PidConfig) -> Self {
        Self {
            config,
            integral: 0.,
            last_meas: 0.,
            d_filt: 0.,
            primed: false,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.;
        self.primed = false;
        self.d_filt = 0.;
    }

    pub fn proportional(&self, error: f32) -> f32 {
        self.config.kp * error
    }

    pub fn integral(&mut self, error: f32) -> f32 {
        self.integral += error * DT;
        self.integral = self
            .integral
            .clamp(-self.config.integral_limit, self.config.integral_limit);
        self.config.ki * self.integral
    }

    pub fn derivitive(&mut self, meas: f32) -> f32 {
        if !self.primed {
            return 0.0;
        }
        let raw = -(meas - self.last_meas) / DT;
        self.d_filt += self.config.d_alpha * (raw - self.d_filt);

        self.config.kd * self.d_filt
    }

    pub fn update(&mut self, setpoint: f32, meas: f32) -> f32 {
        let error = setpoint - meas;

        let p = self.proportional(error);
        let i = self.integral(error);
        let d = self.derivitive(meas);

        self.last_meas = meas;
        self.primed = true;

        let output = (p + i + d).clamp(-self.config.output_limit, self.config.output_limit);

        output
    }
}
