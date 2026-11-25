#[derive(Debug, Clone)]
pub struct Telemetery {
    pub pitch: f32,
    pub roll: f32,
}

impl Default for Telemetery {
    fn default() -> Self {
        Self {
            roll: 0.,
            pitch: 0.,
        }
    }
}
