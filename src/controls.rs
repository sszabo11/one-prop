pub struct Controls {
    pub throttle: f32,
    pub yaw: f32,
    pub y: f32,
    pub x: f32,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            x: 0.,
            y: 0.,
            yaw: 0.,
            throttle: 0.,
        }
    }
}
