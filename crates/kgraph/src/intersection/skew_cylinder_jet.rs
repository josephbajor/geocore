//! Third-order scalar jets for the procedural skew-cylinder evaluator.

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct Jet {
    pub(super) d: [f64; 4],
}

impl Jet {
    pub(super) const fn constant(value: f64) -> Self {
        Self {
            d: [value, 0.0, 0.0, 0.0],
        }
    }

    pub(super) fn derivative(self) -> Self {
        Self {
            d: [self.d[1], self.d[2], self.d[3], 0.0],
        }
    }

    pub(super) fn sqrt(self) -> Self {
        let value = self.d[0].sqrt();
        let first = self.d[1] / (2.0 * value);
        let second = (self.d[2] - 2.0 * first * first) / (2.0 * value);
        let third = (self.d[3] - 6.0 * first * second) / (2.0 * value);
        Self {
            d: [value, first, second, third],
        }
    }

    pub(super) fn reciprocal(self) -> Self {
        let inverse = 1.0 / self.d[0];
        let inverse2 = inverse * inverse;
        let inverse3 = inverse2 * inverse;
        let inverse4 = inverse3 * inverse;
        Self {
            d: [
                inverse,
                -self.d[1] * inverse2,
                2.0 * self.d[1] * self.d[1] * inverse3 - self.d[2] * inverse2,
                -6.0 * self.d[1] * self.d[1] * self.d[1] * inverse4
                    + 6.0 * self.d[1] * self.d[2] * inverse3
                    - self.d[3] * inverse2,
            ],
        }
    }
}

impl core::ops::Add for Jet {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            d: core::array::from_fn(|index| self.d[index] + rhs.d[index]),
        }
    }
}

impl core::ops::Sub for Jet {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            d: core::array::from_fn(|index| self.d[index] - rhs.d[index]),
        }
    }
}

impl core::ops::Neg for Jet {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            d: self.d.map(core::ops::Neg::neg),
        }
    }
}

impl core::ops::Mul for Jet {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            d: [
                self.d[0] * rhs.d[0],
                self.d[1] * rhs.d[0] + self.d[0] * rhs.d[1],
                self.d[2] * rhs.d[0] + 2.0 * self.d[1] * rhs.d[1] + self.d[0] * rhs.d[2],
                self.d[3] * rhs.d[0]
                    + 3.0 * self.d[2] * rhs.d[1]
                    + 3.0 * self.d[1] * rhs.d[2]
                    + self.d[0] * rhs.d[3],
            ],
        }
    }
}

impl core::ops::Mul<f64> for Jet {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            d: self.d.map(|value| value * rhs),
        }
    }
}

impl core::ops::Div<f64> for Jet {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        self * (1.0 / rhs)
    }
}
