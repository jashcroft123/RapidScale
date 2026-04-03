pub trait Filter {
    fn add(&mut self, value: i32) -> i32;
    fn reset(&mut self);
    fn init_to(&mut self, value: i32); // Seed the filter with an initial value
    fn is_saturated(&self) -> bool;
}

// ------------------------
// SMA: Simple Moving Average
// ------------------------
pub struct SMA<const N: usize> {
    buffer: [i32; N],
    index: usize,
    sum: i64,
    count: usize,
}

impl<const N: usize> SMA<N> {
    pub const fn new() -> Self {
        Self {
            buffer: [0; N],
            index: 0,
            sum: 0,
            count: 0,
        }
    }
}

impl<const N: usize> Filter for SMA<N> {
    fn add(&mut self, value: i32) -> i32 {
        if self.count < N {
            self.count += 1;
        } else {
            self.sum -= self.buffer[self.index] as i64;
        }
        self.buffer[self.index] = value;
        self.sum += value as i64;
        self.index = (self.index + 1) % N;
        (self.sum / self.count as i64) as i32
    }

    fn reset(&mut self) {
        self.buffer = [0; N];
        self.index = 0;
        self.sum = 0;
        self.count = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.buffer = [value; N];
        self.sum = value as i64 * N as i64;
        self.index = 0;
        self.count = N;
    }

    fn is_saturated(&self) -> bool {
        self.count == N
    }
}

// ------------------------
// EMA: Exponential Moving Average
// ------------------------
pub struct EMA<const N: usize> {
    pub alpha: f32,
    current: Option<f32>,
    count: usize,
}

impl<const N: usize> EMA<N> {
    pub fn new() -> Self {
        let alpha = 2.0 / (N as f32 + 1.0);
        Self {
            alpha,
            current: None,
            count: 0,
        }
    }

    pub fn with_alpha(alpha: f32) -> Self {
        Self {
            alpha,
            current: None,
            count: 0,
        }
    }
}

impl<const N: usize> Filter for EMA<N> {
    fn add(&mut self, value: i32) -> i32 {
        let val_f = value as f32;
        let next = match self.current {
            Some(curr) => curr + self.alpha * (val_f - curr),
            None => val_f,
        };
        self.current = Some(next);
        if self.count < N {
            self.count += 1;
        }
        next as i32
    }

    fn reset(&mut self) {
        self.current = None;
        self.count = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.current = Some(value as f32);
        self.count = N;
    }

    fn is_saturated(&self) -> bool {
        self.count >= N
    }
}

// ------------------------
// Median Filter
// ------------------------
pub struct Median<const N: usize> {
    buffer: [i32; N],
    index: usize,
    count: usize,
}

impl<const N: usize> Median<N> {
    pub const fn new() -> Self {
        Self {
            buffer: [0; N],
            index: 0,
            count: 0,
        }
    }
}

impl<const N: usize> Filter for Median<N> {
    fn add(&mut self, value: i32) -> i32 {
        self.buffer[self.index] = value;
        self.index = (self.index + 1) % N;
        if self.count < N {
            self.count += 1;
        }

        let mut temp = [0; N];
        temp[..self.count].copy_from_slice(&self.buffer[..self.count]);
        let slice = &mut temp[..self.count];
        slice.sort_unstable();

        if self.count % 2 == 1 {
            slice[self.count / 2]
        } else {
            let mid = self.count / 2;
            (slice[mid - 1] + slice[mid]) / 2
        }
    }

    fn reset(&mut self) {
        self.buffer = [0; N];
        self.index = 0;
        self.count = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.buffer = [value; N];
        self.index = 0;
        self.count = N;
    }

    fn is_saturated(&self) -> bool {
        self.count == N
    }
}

// ------------------------
// Hampel Filter: Outlier Rejection via Median Absolute Deviation
// ------------------------
pub struct HampelFilter<const N: usize> {
    buffer: [i32; N],
    index: usize,
    count: usize,
    sigma_threshold: f32,
}

impl<const N: usize> HampelFilter<N> {
    pub const fn new(sigma_threshold: f32) -> Self {
        Self {
            buffer: [0; N],
            index: 0,
            count: 0,
            sigma_threshold,
        }
    }
}

impl<const N: usize> Filter for HampelFilter<N> {
    fn add(&mut self, value: i32) -> i32 {
        self.buffer[self.index] = value;
        self.index = (self.index + 1) % N;
        if self.count < N {
            self.count += 1;
        }

        if self.count < 3 {
            return value;
        }

        // Calculate median
        let mut sorted = [0; N];
        sorted[..self.count].copy_from_slice(&self.buffer[..self.count]);
        let slice = &mut sorted[..self.count];
        slice.sort_unstable();
        let median = slice[self.count / 2];

        // Calculate MAD (Median Absolute Deviation)
        let mut abs_diffs = [0; N];
        for i in 0..self.count {
            abs_diffs[i] = (self.buffer[i] - median).abs();
        }
        let mad_slice = &mut abs_diffs[..self.count];
        mad_slice.sort_unstable();
        let mad = mad_slice[self.count / 2] as f32;

        // Threshold = sigma_threshold * (mad * 1.4826)
        let std_dev = mad * 1.4826;
        let threshold = self.sigma_threshold * std_dev;

        if (value - median).abs() as f32 > threshold && threshold > 0.0 {
            median // Replace outlier with median
        } else {
            value
        }
    }

    fn reset(&mut self) {
        self.count = 0;
        self.index = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.buffer = [value; N];
        self.count = N;
        self.index = 0;
    }

    fn is_saturated(&self) -> bool {
        self.count == N
    }
}

// ------------------------
// Kalman Filter (1D)
// ------------------------
pub struct KalmanFilter {
    x: f32, // State estimate
    p: f32, // Estimate covariance
    q: f32, // Process noise covariance
    r: f32, // Measurement noise covariance
    initialized: bool,
}

impl KalmanFilter {
    pub const fn new(q: f32, r: f32) -> Self {
        Self {
            x: 0.0,
            p: 1.0,
            q,
            r,
            initialized: false,
        }
    }
}

impl Filter for KalmanFilter {
    fn add(&mut self, value: i32) -> i32 {
        if !self.initialized {
            self.x = value as f32;
            self.initialized = true;
            return value;
        }

        // Prediction
        self.p = self.p + self.q;

        // Measurement Update
        let k = self.p / (self.p + self.r);
        self.x = self.x + k * (value as f32 - self.x);
        self.p = (1.0 - k) * self.p;

        self.x as i32
    }

    fn reset(&mut self) {
        self.initialized = false;
        self.p = 1.0;
    }

    fn init_to(&mut self, value: i32) {
        self.x = value as f32;
        self.p = 0.1; // Low covariance because we're seeding it
        self.initialized = true;
    }

    fn is_saturated(&self) -> bool {
        self.initialized
    }
}

// ------------------------
// Notch Filter (Biquad IIR)
// Tuned for 50Hz @ 320 SPS
// ------------------------
pub struct NotchFilter {
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
    // Coefficients
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
}

impl NotchFilter {
    /// Create a new notch filter for 50Hz @ 320 SPS
    pub const fn new_50hz_320sps() -> Self {
        // Calculated for Q=10
        Self {
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
            b0: 0.9602, b1: -1.0667, b2: 0.9602,
            a1: -1.0667, a2: 0.9203,
        }
    }
}

impl Filter for NotchFilter {
    fn add(&mut self, value: i32) -> i32 {
        let x = value as f32;
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;

        y as i32
    }

    fn reset(&mut self) {
        self.x1 = 0.0; self.x2 = 0.0;
        self.y1 = 0.0; self.y2 = 0.0;
    }

    fn init_to(&mut self, value: i32) {
        let val_f = value as f32;
        self.x1 = val_f;
        self.x2 = val_f;
        self.y1 = val_f;
        self.y2 = val_f;
    }

    fn is_saturated(&self) -> bool {
        true
    }
}

// ------------------------
// Savitzky-Golay (7-point, 2nd Order)
// ------------------------
pub struct SavitzkyGolay7 {
    buffer: [i32; 7],
    index: usize,
    count: usize,
}

impl SavitzkyGolay7 {
    pub const fn new() -> Self {
        Self {
            buffer: [0; 7],
            index: 0,
            count: 0,
        }
    }
}

impl Filter for SavitzkyGolay7 {
    fn add(&mut self, value: i32) -> i32 {
        self.buffer[self.index] = value;
        self.index = (self.index + 1) % 7;
        if self.count < 7 {
            self.count += 1;
            return value;
        }

        // Causal SG-7 endpoint coefficients:
        // (33*x0 + 18*x-1 + 7*x-2 + 0*x-3 - 3*x-4 - 2*x-5 + 3*x-6) / 56
        let x0 = self.buffer[(self.index + 6) % 7] as i64;
        let x1 = self.buffer[(self.index + 5) % 7] as i64;
        let x2 = self.buffer[(self.index + 4) % 7] as i64;
        let x3 = self.buffer[(self.index + 3) % 7] as i64;
        let x4 = self.buffer[(self.index + 2) % 7] as i64;
        let x5 = self.buffer[(self.index + 1) % 7] as i64;
        let x6 = self.buffer[(self.index + 0) % 7] as i64;

        let y = (33 * x0 + 18 * x1 + 7 * x2 + 0 * x3 - 3 * x4 - 2 * x5 + 3 * x6) / 56;
        y as i32
    }

    fn reset(&mut self) {
        self.count = 0;
        self.index = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.buffer = [value; 7];
        self.count = 7;
        self.index = 0;
    }

    fn is_saturated(&self) -> bool {
        self.count == 7
    }
}

// ------------------------
// Filter Chain: stack filters
// ------------------------
pub struct FilterStack<'a> {
    filters: &'a mut [&'a mut dyn Filter],
}

impl<'a> FilterStack<'a> {
    pub fn new(filters: &'a mut [&'a mut dyn Filter]) -> Self {
        Self { filters }
    }
}

impl<'a> Filter for FilterStack<'a> {
    fn add(&mut self, value: i32) -> i32 {
        let mut v = value;
        for f in self.filters.iter_mut() {
            v = f.add(v);
        }
        v
    }

    fn reset(&mut self) {
        for f in self.filters.iter_mut() {
            f.reset();
        }
    }

    fn init_to(&mut self, value: i32) {
        for f in self.filters.iter_mut() {
            f.init_to(value);
        }
    }

    fn is_saturated(&self) -> bool {
        self.filters.iter().all(|f| f.is_saturated())
    }
}
