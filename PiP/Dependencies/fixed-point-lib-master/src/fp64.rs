use crate::FP_SCALE;
use std::fmt::Debug;
use std::ops::{
    Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign, Rem, RemAssign
};

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct Fp64(i64);


impl Fp64 {
    pub const ZERO: Fp64 = Fp64(0);
    pub const ONE: Fp64 = Fp64(1 << FP_SCALE);
    pub const TWO: Fp64 = Fp64(2 << FP_SCALE);
    pub const THREE: Fp64 = Fp64(3 << FP_SCALE);

    pub const PI: Fp64 = Fp64((std::f64::consts::PI * (1 << FP_SCALE) as f64) as i64);
    pub const PI2: Fp64 = Fp64((std::f64::consts::PI * 2.0 * (1 << FP_SCALE) as f64) as i64);
    pub const HALF_PI: Fp64 = Fp64((std::f64::consts::PI / 2.0 * (1 << FP_SCALE) as f64) as i64);

    const THREE_FAC: Fp64 = Fp64(6 << FP_SCALE);
    const FIVE_FAC: Fp64 = Fp64(120 << FP_SCALE);
    const SEVEN_FAC: Fp64 = Fp64(5040 << FP_SCALE);
    

    pub fn new() -> Self {
        Self(0)
    }

    pub fn pow(self, exp: usize) -> Fp64 {
        // TODO: Optimise?
        let mut result = Self::ONE;
        for _ in 0..exp {
            result *= self;
        }
        result
    }

    pub fn sin(self) -> Fp64 {
        let wrapped = ((self + Fp64::PI) % Fp64::PI2) - Fp64::PI;
        wrapped 
            - wrapped.pow(3)/Fp64::THREE_FAC 
            + wrapped.pow(5)/Fp64::FIVE_FAC
            - wrapped.pow(7)/Fp64::SEVEN_FAC
    }
    
    pub fn cos(self) -> Fp64 {
        let wrapped = ((self + Fp64::PI + Fp64::HALF_PI) % Fp64::PI2) - Fp64::PI;
        wrapped 
            - wrapped.pow(3)/Fp64::THREE_FAC 
            + wrapped.pow(5)/Fp64::FIVE_FAC
            - wrapped.pow(7)/Fp64::SEVEN_FAC
    }

    fn estimate_sqrt(self, iterations: usize) -> Fp64 {
        let mut est = Fp64::ONE;
        for _ in 0..iterations {
            let min = self / est;
            let max = est;
            est = (min + max).half();
        }
        est
    }

    pub fn reciprocal_sqrt(self, iterations: usize) -> Fp64 {
        let mut est = Fp64::ONE / self.estimate_sqrt(7);
        for _ in 0..iterations {
            let coefficient = est.half();
            est = (Fp64::THREE - self * est * est) * coefficient;
        }
        est
    }

    pub fn sqrt(self, iterations: usize) -> Fp64 {
        let reciprocal = self.reciprocal_sqrt(iterations);
        return Fp64::ONE / reciprocal;
    }

    pub fn half(self) -> Fp64 {
        Fp64(self.0 >> 1)
    }

    pub fn double(self) -> Fp64 {
        Fp64(self.0 << 1)
    }
}

impl From<i32> for Fp64 {
    fn from(v: i32) -> Self {
        Self(v as i64 * (1 << FP_SCALE))
    }
}

impl From<Fp64> for i32 {
    fn from(fp: Fp64) -> i32 {
        (fp.0 / (1 << FP_SCALE)) as i32
    }
}

impl From<i64> for Fp64 {
    fn from(v: i64) -> Self {
        Self(v * (1 << FP_SCALE))
    }
}

impl From<Fp64> for i64 {
    fn from(fp: Fp64) -> i64 {
        fp.0 / (1 << FP_SCALE)
    }
}

impl From<f32> for Fp64 {
    fn from(v: f32) -> Self {
        Self((v * (1 << FP_SCALE) as f32) as i64)
    }
}

impl From<Fp64> for f32 {
    fn from(fp: Fp64) -> f32 {
        fp.0 as f32 / (1 << FP_SCALE) as f32
    }
}

impl From<f64> for Fp64 {
    fn from(v: f64) -> Self {
        Self((v * (1 << FP_SCALE) as f64) as i64)
    }
}

impl From<Fp64> for f64 {
    fn from(fp: Fp64) -> f64 {
        fp.0 as f64 / (1 << FP_SCALE) as f64
    }
}

impl Add<Fp64> for Fp64 {
    type Output = Self;
    fn add(self, rhs: Fp64) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign<Fp64> for Fp64 {
    fn add_assign(&mut self, rhs: Fp64) {
        self.0 += rhs.0;
    }
}

impl Div<Fp64> for Fp64 {
    type Output = Self;
    fn div(self, rhs: Fp64) -> Self::Output {
        let lhs_internal = self.0 as i128;
        let rhs_internal = rhs.0 as i128;
        Self(((lhs_internal * (1 << FP_SCALE as i128)) / rhs_internal) as i64)
    }
}

impl DivAssign<Fp64> for Fp64 {
    fn div_assign(&mut self, rhs: Fp64) {
        *self = *self / rhs;
    }
}

impl Mul<Fp64> for Fp64 {
    type Output = Self;
    fn mul(self, rhs: Fp64) -> Self::Output {
        let res =
            ((self.0 as i128) * (rhs.0) as i128) / (1 << FP_SCALE) as i128;
        Self(res as i64)
    }
}

impl MulAssign<Fp64> for Fp64 {
    fn mul_assign(&mut self, rhs: Fp64) {
        *self = *self * rhs;
    }
}

impl Sub<Fp64> for Fp64 {
    type Output = Self;
    fn sub(self, rhs: Fp64) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl SubAssign<Fp64> for Fp64 {
    fn sub_assign(&mut self, rhs: Fp64) {
        self.0 -= rhs.0;
    }
}

impl Neg for Fp64 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self(-self.0)
    }
}

impl Rem<Fp64> for Fp64 {
    type Output = Self;
    fn rem(self, rhs: Fp64) -> Self::Output {
        Self(self.0 % rhs.0)
    }
}

impl RemAssign<Fp64> for Fp64 {
    fn rem_assign(&mut self, rhs: Fp64) {
        *self = Self(self.0 % rhs.0);
    }
}

#[cfg(test)]
mod tests {
    use crate::fp64::Fp64;

    #[test]
    fn into_i64() {
        let v = Fp64::from(413.6969);
        let i = i64::from(v);;
        assert_eq!(i, 413);
    }

    #[test]
    fn into_f64() {
        let v = Fp64::from(413.6969);
        let f: f64 = v.into();
        assert!(f > 413.6 && f < 413.7);
    }

    #[test]
    fn sqrt() {
        let iterations = 3;
        let test_value: f64 = 30246.54321;
        let sqrt = test_value.sqrt();
        let min_bound = sqrt - sqrt * 0.05;
        let max_bound = sqrt + sqrt * 0.05;

        let v = Fp64::from(test_value);
        let f: f64 = v.sqrt(iterations).into();
        assert!(
            f > min_bound && f < max_bound,
            "Assertion failed: {} == {}",
            f,
            sqrt
        );
    }

    #[test]
    fn add() {
        let v1 = Fp64::from(2.5);
        let v2 = Fp64::from(7.5);
        let result = v1 + v2;
        assert_eq!(f64::from(result), 10.0);
    }

    #[test]
    fn add_assign() {
        let mut v1 = Fp64::from(2.5);
        let v2 = Fp64::from(7.5);
        v1 += v2;
        assert_eq!(f64::from(v1), 10.0);
    }

    #[test]
    fn div() {
        let v1 = Fp64::from(2.5);
        let v2 = Fp64::from(2.0);
        let f: f64 = (v1 / v2).into();
        assert!(f > 1.24 && f < 1.26, "Assertion failed: {} == 1.25", f);
    }

    #[test]
    fn div_assign() {
        let mut v1 = Fp64::from(2.5);
        let v2 = Fp64::from(2.0);
        v1 /= v2;
        let f: f64 = v1.into();
        assert!(f > 1.24 && f < 1.26, "{} != 1.25", f);
    }

    #[test]
    fn mul() {
        let v1 = Fp64::from(2.1);
        let v2 = Fp64::from(2.0);
        let result = v1 * v2;
        let f: f64 = result.into();
        assert!(f > 4.19 && f < 4.21, "{} != 4.2", f);
    }

    #[test]
    fn mul_assign() {
        let mut v1 = Fp64::from(2.1);
        let v2 = Fp64::from(2.0);
        v1 *= v2;
        let f = f64::from(v1);
        assert!(f > 4.19 && f < 4.21, "{} != 4.2", f);
    }

    #[test]
    fn sub() {
        let v1 = Fp64::from(10.0);
        let v2 = Fp64::from(2.5);
        let f: f64 = (v1 - v2).into();
        assert_eq!(f, 7.5);
    }

    #[test]
    fn sub_assign() {
        let mut v1 = Fp64::from(10.0);
        let v2 = Fp64::from(2.5);
        v1 -= v2;
        assert_eq!(f64::from(v1), 7.5);
    }

    #[test]
    fn pow() {
        let v = Fp64::from(10.0);
        let result = v.pow(5);
        assert_eq!(f64::from(result), 100000.0);
    }

    #[test]
    fn rem() {
        let v1 = Fp64::from(1562.5);
        let v2 = Fp64::from(10);
        let result = v1 % v2;
        assert_eq!(f64::from(result), 2.5);
    }

    #[test]
    fn rem_complex() {
        let v1 = Fp64::from(4.2);
        let v2 = Fp64::from(0.8);
        let result = v1 % v2;
        let f = f64::from(result);
        assert!(f > 0.1999 && f < 0.2001, "{} != 0.2", f);
    }

    #[test]
    fn rem_below() {
        let v1 = Fp64::from(3.5);
        let v2 = Fp64::from(5);
        let result = v1 % v2;
        let f = f64::from(result);
        assert_eq!(f, 3.5);
    }


    #[test]
    fn pi() {
        let pi = f64::from(Fp64::PI);
        assert!(pi > 3.1415 && pi < 3.1416, "{} != PI", pi);
    }

    #[test]
    fn pi2() {
        let pi2 = f64::from(Fp64::PI2);
        assert!(pi2 > 3.1415*2.0 && pi2 < 3.1416*2.0, "{} != PI*2", pi2);
    }

    #[test]
    fn sin_start() {
        let result = Fp64::sin(Fp64::ZERO);
        let f = f64::from(result);
        assert_eq!(f, 0.0);
    }

    #[test]
    fn sin_peak() {
        let result = Fp64::sin(Fp64::HALF_PI);
        let f = f64::from(result);

        assert!(f > 0.999 && f < 1.001, "{} != 1.0", f);
    }

    #[test]
    fn sin_edge() {
        let result = Fp64::sin(Fp64::PI);
        let f = f64::from(result);

        // Approximation is worse the further the input is from 0.0
        assert!(f > -0.1 && f < 0.1, "{} != 0.0", f);
    }

    #[test]
    fn sin_double() {
        let result = Fp64::sin(Fp64::PI2);
        let f = f64::from(result);
        
        // Approximation is worse the further the input is from 0.0
        assert!(f > -0.1 && f < 0.1, "{} != 0.0", f);
    }

    #[test]
    fn cos_start() {
        let result = Fp64::cos(Fp64::ZERO);
        let f = f64::from(result);
        assert!(f > 0.999 && f < 1.001, "{} != 1.0", f);
    }

    #[test]
    fn cos_min() {
        let result = Fp64::cos(Fp64::HALF_PI);
        let f = f64::from(result);
        assert!(f > -0.1 && f < 0.1, "{} != 0.0", f);
    }

    #[test]
    fn cos_edge() {
        let result = Fp64::cos(Fp64::PI);
        let f = f64::from(result);

        // Approximation is worse the further the input is from 0.0
        assert!(f > -1.1 && f < -0.9, "{} != -1.0", f);
    }

    #[test]
    fn cos_double() {
        let result = Fp64::cos(Fp64::PI2);
        let f = f64::from(result);
        
        // Approximation is worse the further the input is from 0.0
        assert!(f < 1.1 && f > 0.9, "{} != 1.0", f);
    }
}
