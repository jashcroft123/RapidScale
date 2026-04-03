pub trait StabilityDetector {
    /// Returns true if the signal is considered unstable.
    fn is_unstable(&mut self, value: i32) -> bool;
    /// Reset internal state.
    fn reset(&mut self);
    /// Seed the detector with an initial value.
    fn init_to(&mut self, value: i32);
    /// Returns true if the detector has enough data to be valid.
    fn is_saturated(&self) -> bool;
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

impl<const N: usize> StabilityDetector for VarianceDetector<N> {
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

impl StabilityDetector for DifferenceDetector {
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
// Combines multiple sources AND debounces them in one pass.
// ------------------------
pub struct StabilityStack<'a> {
    detectors: &'a mut [&'a mut dyn StabilityDetector],
    required_stable_samples: usize,
    stable_count: usize,
}

impl<'a> StabilityStack<'a> {
    pub fn new(
        detectors: &'a mut [&'a mut dyn StabilityDetector],
        required_stable_samples: usize,
    ) -> Self {
        Self {
            detectors,
            required_stable_samples,
            stable_count: 0,
        }
    }
}

impl<'a> StabilityDetector for StabilityStack<'a> {
    fn is_unstable(&mut self, value: i32) -> bool {
        // 1. Check ALL detectors (OR logic for instability)
        let mut any_unstable = false;
        for d in self.detectors.iter_mut() {
            if d.is_unstable(value) {
                any_unstable = true;
            }
        }

        // 2. Apply Debounce Logic
        if any_unstable {
            self.stable_count = 0;
            true // Trip Fast mode immediately
        } else {
            if self.stable_count < self.required_stable_samples {
                self.stable_count += 1;
                true // Still in settling period
            } else {
                false // Unlocked Stability!
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
        // If we seed the detectors, we consider the debounce "charged" too
        self.stable_count = self.required_stable_samples;
    }

    fn is_saturated(&self) -> bool {
        self.detectors.iter().all(|d| d.is_saturated())
            && self.stable_count >= self.required_stable_samples
    }
}
