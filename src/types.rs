#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScaleMode {
    Fast,
    Stable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScaleState {
    Tare,
    Reading,
}

#[derive(Clone, Copy)]
pub struct DisplayData {
    pub value: f32,
    pub tare_flag: bool,
}
