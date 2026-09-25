#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StabilityLevel {
    Unstable,
    Settling,
    Stable,
}

#[allow(dead_code)]
pub trait StabilityDetector {
    fn check(&mut self, value: i32) -> StabilityLevel;
    fn reset(&mut self);
    fn init_to(&mut self, value: i32);
    fn is_saturated(&self) -> bool;
}

#[allow(dead_code)]
pub trait StabilitySource {
    fn is_unstable(&mut self, value: i32) -> bool;
    fn reset(&mut self);
    fn init_to(&mut self, value: i32);
    fn is_saturated(&self) -> bool;
    fn variance(&self) -> Option<i64> {
        None
    }
    fn set_variance_threshold(&mut self, _threshold: i64) {}
    fn set_difference_threshold(&mut self, _threshold: i32) {}
}

// ------------------------
// Median Decorator: Rejects impulsive spikes (like Tapping on the table)
// ------------------------
pub struct MedianDecorator<'a, const N: usize> {
    inner: &'a mut dyn StabilitySource,
    buf: [i32; N],
    idx: usize,
    count: usize,
}

impl<'a, const N: usize> MedianDecorator<'a, N> {
    pub fn new(inner: &'a mut dyn StabilitySource) -> Self {
        Self {
            inner,
            buf: [0; N],
            idx: 0,
            count: 0,
        }
    }
}

impl<'a, const N: usize> StabilitySource for MedianDecorator<'a, N> {
    fn is_unstable(&mut self, value: i32) -> bool {
        self.buf[self.idx] = value;
        self.idx = (self.idx + 1) % N;
        if self.count < N {
            self.count += 1;
        }

        // Calculate Median
        let mut sorted = [0; N];
        sorted[..self.count].copy_from_slice(&self.buf[..self.count]);
        let slice = &mut sorted[..self.count];
        slice.sort_unstable();

        let median = if self.count % 2 == 1 {
            slice[self.count / 2]
        } else {
            let mid = self.count / 2;
            (slice[mid - 1] + slice[mid]) / 2
        };

        self.inner.is_unstable(median)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.count = 0;
        self.idx = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.inner.init_to(value);
        self.buf = [value; N];
        self.count = N;
        self.idx = 0;
    }

    fn is_saturated(&self) -> bool {
        self.inner.is_saturated() && self.count == N
    }

    fn variance(&self) -> Option<i64> {
        self.inner.variance()
    }

    fn set_variance_threshold(&mut self, threshold: i64) {
        self.inner.set_variance_threshold(threshold);
    }

    fn set_difference_threshold(&mut self, threshold: i32) {
        self.inner.set_difference_threshold(threshold);
    }
}

// ------------------------
// Deadband Decorator: Suppresses noise by holding the last value if change is small
// ------------------------
#[allow(dead_code)]
pub struct DeadbandDecorator<'a> {
    inner: &'a mut dyn StabilitySource,
    deadband: i32,
    last_center: i32,
}

impl<'a> DeadbandDecorator<'a> {
    #[allow(dead_code)]
    pub fn new(inner: &'a mut dyn StabilitySource, deadband: i32) -> Self {
        Self {
            inner,
            deadband,
            last_center: 0,
        }
    }
}

impl<'a> StabilitySource for DeadbandDecorator<'a> {
    fn is_unstable(&mut self, value: i32) -> bool {
        let diff = (value - self.last_center).abs();

        let value_to_use = if diff < self.deadband {
            self.last_center
        } else {
            self.last_center = value;
            value
        };

        self.inner.is_unstable(value_to_use)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.last_center = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.inner.init_to(value);
        self.last_center = value;
    }

    fn is_saturated(&self) -> bool {
        self.inner.is_saturated()
    }
}

// ------------------------
// Variance Detector (Base Source) - O(1) Running Variance with Exact Numerator
// ------------------------
const VARIANCE_CONFIRM_SAMPLES: usize = 8;

pub struct VarianceDetector<const N: usize> {
    buf: [i32; N],
    idx: usize,
    count: usize,
    sum: i64,
    sum_sq: i64,
    over_threshold_count: usize,
    threshold: i64,
}

impl<const N: usize> VarianceDetector<N> {
    pub const fn new(threshold: i64) -> Self {
        Self {
            buf: [0; N],
            idx: 0,
            count: 0,
            sum: 0,
            sum_sq: 0,
            over_threshold_count: 0,
            threshold,
        }
    }
}

impl<const N: usize> StabilitySource for VarianceDetector<N> {
    fn is_unstable(&mut self, v: i32) -> bool {
        let v64 = v as i64;

        if self.count < N {
            self.buf[self.idx] = v;
            self.sum += v64;
            self.sum_sq += v64 * v64;
            self.idx = (self.idx + 1) % N;
            self.count += 1;
            return true;
        }

        let old = self.buf[self.idx] as i64;
        self.buf[self.idx] = v;
        self.idx = (self.idx + 1) % N;

        self.sum = self.sum - old + v64;
        self.sum_sq = self.sum_sq - (old * old) + (v64 * v64);

        // Compare the scaled sum of squares in i128. Dividing the raw ADC
        // offset down to a mean first truncates away the noise, and the
        // i64 products overflow once counts leave the 24-bit range.
        let n = N as i128;
        let numerator = n * i128::from(self.sum_sq) - i128::from(self.sum) * i128::from(self.sum);
        let limit = i128::from(self.threshold) * n * n;
        if numerator > limit {
            self.over_threshold_count = self.over_threshold_count.saturating_add(1);
            self.over_threshold_count >= VARIANCE_CONFIRM_SAMPLES
        } else {
            self.over_threshold_count = 0;
            false
        }
    }

    fn reset(&mut self) {
        self.buf = [0; N];
        self.count = 0;
        self.idx = 0;
        self.sum = 0;
        self.sum_sq = 0;
        self.over_threshold_count = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.buf = [value; N];
        self.count = N;
        self.idx = 0;
        let v64 = value as i64;
        self.sum = v64 * N as i64;
        self.sum_sq = (v64 * v64) * N as i64;
        self.over_threshold_count = 0;
    }

    fn is_saturated(&self) -> bool {
        self.count == N
    }

    fn variance(&self) -> Option<i64> {
        if self.count < N || N == 0 {
            return None;
        }
        let n = N as i128;
        let numerator = n * i128::from(self.sum_sq)
            - i128::from(self.sum) * i128::from(self.sum);
        Some((numerator / (n * n)) as i64)
    }

    fn set_variance_threshold(&mut self, threshold: i64) {
        self.threshold = threshold;
    }
}

// ------------------------
// Difference Detector (Base Source)
// ------------------------
pub struct DifferenceDetector {
    last_value: Option<i32>,
    threshold: i32,
}

impl DifferenceDetector {
    pub const fn new(threshold: i32) -> Self {
        Self {
            last_value: None,
            threshold,
        }
    }
}

impl StabilitySource for DifferenceDetector {
    fn is_unstable(&mut self, value: i32) -> bool {
        let unstable = match self.last_value {
            Some(v) => (value - v).abs() > self.threshold,
            None => false,
        };
        self.last_value = Some(value);
        unstable
    }

    fn reset(&mut self) {
        self.last_value = None;
    }

    fn init_to(&mut self, value: i32) {
        self.last_value = Some(value);
    }

    fn is_saturated(&self) -> bool {
        self.last_value.is_some()
    }

    fn set_difference_threshold(&mut self, threshold: i32) {
        self.threshold = threshold;
    }
}

// ------------------------
// StabilityStack: THE SINGLE DECORATOR
// Combines multiple sources AND handles multi-stage debouncing.
// ------------------------
pub struct StabilityStack<'a> {
    detectors: &'a mut [&'a mut dyn StabilitySource],
    stable_threshold: usize,
    stable_count: usize,
}

impl<'a> StabilityStack<'a> {
    pub fn new(
        detectors: &'a mut [&'a mut dyn StabilitySource],
        stable_threshold: usize,
    ) -> Self {
        Self {
            detectors,
            stable_threshold,
            stable_count: 0,
        }
    }

    /// Calibrate noise rejection from the empty-pan tare window.
    pub fn calibrate_noise_thresholds(&mut self) -> Option<(i64, i64, i32)> {
        let measured = self.detectors.iter().find_map(|detector| detector.variance())?;
        const MIN_THRESHOLD: i64 = 3_500;
        const MAX_THRESHOLD: i64 = 1_000_000;
        let threshold = measured.saturating_mul(4).clamp(MIN_THRESHOLD, MAX_THRESHOLD);
        let jump_threshold = (libm::sqrtf(measured as f32) * 4.0) as i32;
        let jump_threshold = jump_threshold.clamp(300, 5_000);
        for detector in self.detectors.iter_mut() {
            detector.set_variance_threshold(threshold);
            detector.set_difference_threshold(jump_threshold);
        }
        Some((measured, threshold, jump_threshold))
    }

    pub fn measured_variance(&self) -> Option<i64> {
        self.detectors.iter().find_map(|detector| detector.variance())
    }
}

impl<'a> StabilityDetector for StabilityStack<'a> {
    fn check(&mut self, value: i32) -> StabilityLevel {
        // 1. Check ALL detectors
        let mut any_unstable = false;
        for d in self.detectors.iter_mut() {
            if d.is_unstable(value) {
                any_unstable = true;
            }
        }

        // A detector firing is motion. Quiet samples are Settling until the
        // debounce count locks Stable, so the fast path is not used to wait.
        if any_unstable {
            self.stable_count = 0;
            StabilityLevel::Unstable
        } else {
            self.stable_count += 1;
            if self.stable_count >= self.stable_threshold {
                StabilityLevel::Stable
            } else {
                StabilityLevel::Settling
            }
        }
    }

    fn reset(&mut self) {
        for d in self.detectors.iter_mut() {
            d.reset();
        }
        self.stable_count = 0;
    }

    fn init_to(&mut self, value: i32) {
        for d in self.detectors.iter_mut() {
            d.init_to(value);
        }
        self.stable_count = self.stable_threshold;
    }

    fn is_saturated(&self) -> bool {
        self.detectors.iter().all(|d| d.is_saturated())
            && self.stable_count >= self.stable_threshold
    }
}
