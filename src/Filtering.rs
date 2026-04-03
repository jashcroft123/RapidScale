pub trait Filter {
    fn add(&mut self, value: i32) -> i32;
    fn reset(&mut self);
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

    fn is_saturated(&self) -> bool {
        self.count >= N
    }
}

// ------------------------
// Gaussian Weighted Moving Average
// ------------------------
pub struct Gaussian<const N: usize> {
    buffer: [i32; N],
    weights: [f32; N],
    index: usize,
    count: usize,
}

impl<const N: usize> Gaussian<N> {
    pub fn new(sigma: f32) -> Self {
        let mut weights = [0.0; N];
        let mut sum = 0.0;
        let center = (N - 1) as f32 / 2.0;

        for i in 0..N {
            let x = i as f32 - center;
            weights[i] = libm::expf(-(x * x) / (2.0 * sigma * sigma));
            sum += weights[i];
        }
        for i in 0..N {
            weights[i] /= sum;
        }

        Self {
            buffer: [0; N],
            weights,
            index: 0,
            count: 0,
        }
    }
}

impl<const N: usize> Filter for Gaussian<N> {
    fn add(&mut self, value: i32) -> i32 {
        self.buffer[self.index] = value;
        self.index = (self.index + 1) % N;
        if self.count < N {
            self.count += 1;
        }

        let mut output = 0.0;
        for i in 0..self.count {
            let buf_idx = (self.index + N - 1 - i) % N;
            output += self.buffer[buf_idx] as f32 * self.weights[i];
        }

        if self.count < N {
            let mut current_sum = 0.0;
            for i in 0..self.count {
                current_sum += self.weights[i];
            }
            output /= current_sum;
        }

        output as i32
    }

    fn reset(&mut self) {
        self.buffer = [0; N];
        self.index = 0;
        self.count = 0;
    }

    fn is_saturated(&self) -> bool {
        self.count == N
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

    fn is_saturated(&self) -> bool {
        self.count == N
    }
}

// ------------------------
// Filter Chain: stack filters
// ------------------------
pub struct DualFilter<A: Filter, B: Filter> {
    first: A,
    second: B,
}

impl<A: Filter, B: Filter> DualFilter<A, B> {
    pub fn new(first: A, second: B) -> Self {
        Self { first, second }
    }
}

impl<A: Filter, B: Filter> Filter for DualFilter<A, B> {
    fn add(&mut self, value: i32) -> i32 {
        let intermediate = self.first.add(value);
        self.second.add(intermediate)
    }

    fn reset(&mut self) {
        self.first.reset();
        self.second.reset();
    }

    fn is_saturated(&self) -> bool {
        self.first.is_saturated() && self.second.is_saturated()
    }
}

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

    fn is_saturated(&self) -> bool {
        self.filters.iter().all(|f| f.is_saturated())
    }
}
