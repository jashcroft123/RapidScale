#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StabilityLevel {
    Unstable,
    Settling,
    Stable,
}

pub trait StabilityDetector {
    fn check(&mut self, value: i32) -> StabilityLevel;
    fn reset(&mut self);
    fn init_to(&mut self, value: i32);
    fn is_saturated(&self) -> bool;
}

pub trait StabilitySource {
    fn is_unstable(&mut self, value: i32) -> bool;
    fn reset(&mut self);
    fn init_to(&mut self, value: i32);
    fn is_saturated(&self) -> bool;
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
}

// ------------------------
// Deadband Decorator: Suppresses noise by holding the last value if change is small
// ------------------------
pub struct DeadbandDecorator<'a> {
    inner: &'a mut dyn StabilitySource,
    deadband: i32,
    last_center: i32,
}

impl<'a> DeadbandDecorator<'a> {
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
// Variance Detector (Base Source)
// ------------------------
pub struct VarianceDetector<const N: usize> {
    buf: [i32; N],
    idx: usize,
    count: usize,
    threshold: i64,
}

impl<const N: usize> VarianceDetector<N> {
    pub const fn new(threshold: i64) -> Self {
        Self {
            buf: [0; N],
            idx: 0,
            count: 0,
            threshold,
        }
    }
}

impl<const N: usize> StabilitySource for VarianceDetector<N> {
    fn is_unstable(&mut self, v: i32) -> bool {
        self.buf[self.idx] = v;
        self.idx = (self.idx + 1) % N;
        if self.count < N {
            self.count += 1;
            return true;
        }

        let n = self.count as i64;
        let sum: i64 = self.buf.iter().map(|&x| x as i64).sum();
        let mean = sum / n;

        let variance: i64 = self
            .buf
            .iter()
            .map(|&x| {
                let d = x as i64 - mean;
                d * d
            })
            .sum::<i64>()
            / n;

        variance > self.threshold
    }

    fn reset(&mut self) {
        self.count = 0;
        self.idx = 0;
    }

    fn init_to(&mut self, value: i32) {
        self.buf = [value; N];
        self.count = N;
        self.idx = 0;
    }

    fn is_saturated(&self) -> bool {
        self.count == N
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
}

// ------------------------
// StabilityStack: THE SINGLE DECORATOR
// Combines multiple sources AND handles multi-stage debouncing.
// ------------------------
pub struct StabilityStack<'a> {
    detectors: &'a mut [&'a mut dyn StabilitySource],
    settle_threshold: usize,
    stable_threshold: usize,
    stable_count: usize,
}

impl<'a> StabilityStack<'a> {
    pub fn new(
        detectors: &'a mut [&'a mut dyn StabilitySource],
        settle_threshold: usize,
        stable_threshold: usize,
    ) -> Self {
        Self {
            detectors,
            settle_threshold,
            stable_threshold,
            stable_count: 0,
        }
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

        // 2. State management
        if any_unstable {
            self.stable_count = 0;
            StabilityLevel::Unstable
        } else {
            self.stable_count += 1;
            if self.stable_count >= self.stable_threshold {
                StabilityLevel::Stable
            } else if self.stable_count >= self.settle_threshold {
                StabilityLevel::Settling
            } else {
                StabilityLevel::Unstable 
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
        self.detectors.iter().all(|d| d.is_saturated()) && self.stable_count >= self.stable_threshold
    }
}
