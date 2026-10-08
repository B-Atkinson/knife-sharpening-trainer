use approx::{AbsDiffEq, RelativeEq};
use core::fmt::Formatter;
use core::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

#[derive(Debug)]
pub enum Vec3Error {
    NormTooSmall(Vec3),
    InvalidCoefficients(f32, f32, f32),
    InvalidClampBounds(f32, f32),
}

impl core::fmt::Display for Vec3Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Vec3Error::NormTooSmall(v) => write!(
                f,
                "vector x:{}, y:{}, z:{} produces norm of {} which is too small",
                v.x,
                v.y,
                v.z,
                v.norm()
            ),
            Vec3Error::InvalidCoefficients(x, y, z) => write!(
                f,
                "vector components x:{}, y:{}, z:{} must be finite and produce a finite magnitude",
                x, y, z
            ),
            Vec3Error::InvalidClampBounds(min, max) => write!(
                f,
                "vector clamp bounds min:{} and max:{} must be finite and min must not exceed max",
                min, max
            ),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, other: Self) {
        *self = Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, other: Self) {
        *self = Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;

    fn mul(self, rhs: f32) -> Vec3 {
        Vec3 {
            x: rhs * self.x,
            y: rhs * self.y,
            z: rhs * self.z,
        }
    }
}

impl Mul<Vec3> for f32 {
    type Output = Vec3;

    fn mul(self, vector: Vec3) -> Vec3 {
        vector * self
    }
}

impl Div<f32> for Vec3 {
    type Output = Vec3;

    fn div(self, rhs: f32) -> Vec3 {
        Vec3 {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl AbsDiffEq for Vec3 {
    type Epsilon = f32;

    fn default_epsilon() -> Self::Epsilon {
        f32::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        f32::abs_diff_eq(&self.x, &other.x, epsilon)
            && f32::abs_diff_eq(&self.y, &other.y, epsilon)
            && f32::abs_diff_eq(&self.z, &other.z, epsilon)
    }
}

impl RelativeEq for Vec3 {
    fn default_max_relative() -> Self::Epsilon {
        f32::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        f32::relative_eq(&self.x, &other.x, epsilon, max_relative)
            && f32::relative_eq(&self.y, &other.y, epsilon, max_relative)
            && f32::relative_eq(&self.z, &other.z, epsilon, max_relative)
    }
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    pub fn dot(self, other: Vec3) -> f32 {
        (self.x * other.x) + (self.y * other.y) + (self.z * other.z)
    }

    pub fn cross(self, other: Vec3) -> Self {
        Self {
            x: (self.y * other.z) - (self.z * other.y),
            y: (self.z * other.x) - (self.x * other.z),
            z: (self.x * other.y) - (self.y * other.x),
        }
    }

    pub fn norm_squared(&self) -> f32 {
        self.dot(*self)
    }

    pub fn norm(&self) -> f32 {
        libm::sqrtf(self.norm_squared())
    }

    pub fn normalize(self) -> Result<Self, Vec3Error> {
        if !(self.x.is_finite() && self.y.is_finite() && self.z.is_finite()) {
            return Err(Vec3Error::InvalidCoefficients(self.x, self.y, self.z));
        }

        let norm = self.norm();
        if !norm.is_finite() {
            return Err(Vec3Error::InvalidCoefficients(self.x, self.y, self.z));
        }

        if norm <= 1e-11_f32 {
            return Err(Vec3Error::NormTooSmall(self));
        }

        let normalized = self / norm;
        if !(normalized.x.is_finite() && normalized.y.is_finite() && normalized.z.is_finite()) {
            return Err(Vec3Error::InvalidCoefficients(self.x, self.y, self.z));
        }

        Ok(normalized)
    }

    pub fn component_clamp(self, min: f32, max: f32) -> Result<Self, Vec3Error> {
        if !(self.x.is_finite() && self.y.is_finite() && self.z.is_finite()) {
            return Err(Vec3Error::InvalidCoefficients(self.x, self.y, self.z));
        }
        if !(min.is_finite() && max.is_finite()) || min > max {
            return Err(Vec3Error::InvalidClampBounds(min, max));
        }

        Ok(Self {
            x: self.x.clamp(min, max),
            y: self.y.clamp(min, max),
            z: self.z.clamp(min, max),
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::vec3::{Vec3, Vec3Error};

    #[test]
    fn test_add() {
        let v1 = Vec3::new(1.0f32, 1.0f32, 1.0f32);
        let v2 = Vec3::new(1.0f32, 1.0f32, 1.0f32);
        assert_eq!(v1 + v2, Vec3::new(2.0f32, 2.0f32, 2.0f32));

        let v3 = Vec3::new(0f32, 0f32, 0f32);
        assert_eq!(v1 + v3, v1);

        let v4 = Vec3::new(-1f32, 0f32, 0f32);
        assert_eq!(v1 + v4, Vec3::new(0f32, 1.0f32, 1.0f32));
    }

    #[test]
    fn test_add_assign() {
        let mut v1 = Vec3::new(1.0f32, 2.0f32, 3.0f32);
        v1 += Vec3::new(4.0f32, -2.0f32, 0.5f32);

        assert_eq!(v1, Vec3::new(5.0f32, 0.0f32, 3.5f32));
    }

    #[test]
    fn test_sub() {
        let v1 = Vec3::new(1.0f32, 1.0f32, 1.0f32);
        let v2 = Vec3::new(1.0f32, 1.0f32, 1.0f32);
        assert_eq!(v1 - v2, Vec3::new(0f32, 0f32, 0f32));

        let v3 = Vec3::new(0f32, 0f32, 0f32);
        assert_eq!(v1 - v3, v1);

        let v4 = Vec3::new(-1f32, 0f32, 0f32);
        assert_eq!(v1 - v4, Vec3::new(2f32, 1.0f32, 1.0f32));
    }

    #[test]
    fn test_sub_assign() {
        let mut v1 = Vec3::new(1.0f32, 2.0f32, 3.0f32);
        v1 -= Vec3::new(4.0f32, -2.0f32, 0.5f32);

        assert_eq!(v1, Vec3::new(-3.0f32, 4.0f32, 2.5f32));
    }

    #[test]
    fn test_mul() {
        let v1 = Vec3::new(1.0f32, 1.0f32, 1.0f32);
        assert_eq!(2f32 * v1, Vec3::new(2.0f32, 2.0f32, 2.0f32));
        assert_eq!(v1 * 2f32, Vec3::new(2.0f32, 2.0f32, 2.0f32));

        let v2 = Vec3::new(0f32, 0f32, 0f32);
        assert_eq!(v1 * 0f32, v2);
        assert_eq!(0f32 * v1, v2);

        let v3 = Vec3::new(-1f32, 0f32, 0f32);
        assert_eq!(2f32 * v3, Vec3::new(-2f32, 0f32, 0f32));
        assert_eq!(v3 * 2f32, Vec3::new(-2f32, 0f32, 0f32));
    }

    #[test]
    fn test_div() {
        let v1 = Vec3::new(2.0f32, -4.0f32, 8.0f32);

        assert_eq!(v1 / 2.0f32, Vec3::new(1.0f32, -2.0f32, 4.0f32));
        assert_eq!(v1 / -2.0f32, Vec3::new(-1.0f32, 2.0f32, -4.0f32));
    }

    #[test]
    fn test_dot() {
        let v1 = Vec3::new(1.0f32, 1.0f32, 1.0f32);
        assert_eq!(v1.dot(v1), 3.0f32);

        let v2 = Vec3::new(1.0f32, 2.0f32, 3.0f32);
        assert_eq!(v1.dot(v2), 6.0f32);

        let v3 = Vec3::new(1.0f32, 2.0f32, -3.0f32);
        assert_eq!(v1.dot(v3), 0f32);
    }

    #[test]
    fn test_cross() {
        let vec_i = Vec3::new(1f32, 0f32, 0f32);
        let vec_j = Vec3::new(0f32, 1.0f32, 0f32);
        let vec_k = Vec3::new(0f32, 0f32, 1.0f32);
        let vec_zero = Vec3::new(0f32, 0f32, 0f32);

        assert_eq!(vec_i.cross(vec_j), vec_k);
        assert_eq!(vec_j.cross(vec_k), vec_i);
        assert_eq!(vec_k.cross(vec_i), vec_j);

        assert_eq!(vec_i.cross(vec_i), vec_zero);
        assert_eq!(vec_j.cross(vec_j), vec_zero);
        assert_eq!(vec_k.cross(vec_k), vec_zero);
    }

    #[test]
    fn test_norm_squared() {
        let v1 = Vec3::new(1.0f32, 1.0f32, 1.0f32);
        assert_eq!(v1.norm_squared(), 3f32);

        let v2 = Vec3::new(2f32, -2f32, 3f32);
        let v3 = Vec3::new(2f32, 2f32, 3f32);
        assert_eq!(v2.norm_squared(), v3.norm_squared());
        assert_eq!(v2.norm_squared(), 17f32);
    }

    #[test]
    fn test_norm() {
        let v1 = Vec3::new(3f32, 4f32, 0f32);
        assert_eq!(v1.norm(), 5f32);

        let v2 = Vec3::new(0f32, 0f32, 0f32);
        assert_eq!(v2.norm(), 0f32);

        let v3 = Vec3::new(-3f32, 4f32, 0f32);
        assert_eq!(v3.norm(), 5f32);

        let v4 = Vec3::new(-3f32, -4f32, 0f32);
        assert_eq!(v4.norm(), 5f32);
    }

    #[test]
    fn test_normalize() {
        let v1 = Vec3::new(1f32, 1f32, 1f32);
        let v2 = 2f32 * v1;
        assert!(v1.normalize().is_ok());
        assert!(v2.normalize().is_ok());
        approx::assert_relative_eq!(v1.normalize().unwrap(), v2.normalize().unwrap());

        let v3 = Vec3::new(1e-12_f32, 1e-12_f32, 1e-12_f32);
        assert!(matches!(v3.normalize(), Err(Vec3Error::NormTooSmall(_))));

        let v4 = Vec3::new(0f32, 0f32, 0f32);
        assert!(matches!(v4.normalize(), Err(Vec3Error::NormTooSmall(_))));

        let v5 = Vec3::new(-3f32, 0f32, 4f32);
        assert_eq!(v5.normalize().unwrap(), Vec3::new(-0.6f32, 0f32, 0.8f32));
        assert_eq!(v5.norm(), 5f32);
    }

    #[test]
    fn test_normalize_rejects_non_finite_components() {
        assert!(matches!(
            Vec3::new(f32::NAN, 0.0_f32, 0.0_f32).normalize(),
            Err(Vec3Error::InvalidCoefficients(_, _, _))
        ));
        assert!(matches!(
            Vec3::new(0.0_f32, f32::INFINITY, 0.0_f32).normalize(),
            Err(Vec3Error::InvalidCoefficients(_, _, _))
        ));
    }

    #[test]
    fn test_normalize_rejects_non_finite_norm() {
        assert!(matches!(
            Vec3::new(f32::MAX, f32::MAX, f32::MAX).normalize(),
            Err(Vec3Error::InvalidCoefficients(_, _, _))
        ));
    }

    #[test]
    fn test_component_clamp() {
        let v1 = Vec3::new(-2.0f32, 0.5f32, 3.0f32);
        assert_eq!(
            v1.component_clamp(-1.0f32, 1.0f32).unwrap(),
            Vec3::new(-1.0f32, 0.5f32, 1.0f32)
        );

        let v2 = Vec3::new(-0.25f32, 0.0f32, 0.75f32);
        assert_eq!(v2.component_clamp(-1.0f32, 1.0f32).unwrap(), v2);
    }

    #[test]
    fn test_component_clamp_rejects_invalid_bounds() {
        let vector = Vec3::new(-2.0_f32, 0.5_f32, 3.0_f32);

        assert!(matches!(
            vector.component_clamp(1.0_f32, -1.0_f32),
            Err(Vec3Error::InvalidClampBounds(_, _))
        ));
        assert!(matches!(
            vector.component_clamp(f32::NAN, 1.0_f32),
            Err(Vec3Error::InvalidClampBounds(_, _))
        ));
    }

    #[test]
    fn test_component_clamp_rejects_non_finite_vector() {
        assert!(matches!(
            Vec3::new(f32::INFINITY, 0.0_f32, 0.0_f32).component_clamp(-1.0_f32, 1.0_f32),
            Err(Vec3Error::InvalidCoefficients(_, _, _))
        ));
    }
}
