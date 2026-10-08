//! Hamilton quaternions for attitude estimation and vector rotation.
//!
//! [`Quat`] stores components in `[w, x, y, z]` order. A unit quaternion used
//! as an attitude maps vectors from the current IMU/body frame (`I`) into the
//! calibration/reference frame (`C`) with `q * [0, v] * q.conjugate()`.
//! Quaternion multiplication is the Hamilton product and is not commutative.
//!
//! # Example
//!
//! ```
//! use sharpener_math::quaternion::Quat;
//! use sharpener_math::vec3::Vec3;
//!
//! let attitude_i_to_c = Quat::identity();
//! let body_vector = Vec3::new(1.0, 2.0, 3.0);
//!
//! assert_eq!(attitude_i_to_c.rotate_unit_i_to_c(body_vector), body_vector);
//! ```

use crate::vec3::Vec3;
use approx::{AbsDiffEq, RelativeEq};
use core::fmt::Formatter;
use core::ops::{Add, AddAssign, Div, Mul, Sub, SubAssign};

/// An error produced while validating or normalizing a [`Quat`].
#[derive(Debug)]
pub enum QuatError {
    /// One or more coefficients, or the resulting norm, were non-finite.
    InvalidCoefficients(f32, f32, f32, f32),

    /// The quaternion norm was at or below the normalization safety threshold.
    NormTooSmall(Quat),
}

impl core::fmt::Display for QuatError {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            QuatError::InvalidCoefficients(w, x, y, z) => write!(
                f,
                "candidate coefficients w:{}, x:{}, y:{}, z:{} produces magnitude of {}",
                w,
                x,
                y,
                z,
                libm::sqrtf(w * w + x * x + y * y + z * z)
            ),
            QuatError::NormTooSmall(q) => write!(
                f,
                "quaternion w:{}, x:{}, y:{}, z:{} produces norm of {} which is too small",
                q.w,
                q.x,
                q.y,
                q.z,
                libm::sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
            ),
        }
    }
}

impl Add for Quat {
    type Output = Quat;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for Quat {
    fn add_assign(&mut self, rhs: Self) {
        *self = Self {
            w: self.w + rhs.w,
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Sub for Quat {
    type Output = Quat;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl SubAssign for Quat {
    fn sub_assign(&mut self, rhs: Self) {
        *self = Self {
            w: self.w - rhs.w,
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Mul<Quat> for Quat {
    type Output = Quat;

    fn mul(self, rhs: Quat) -> Quat {
        Self {
            w: (self.w * rhs.w) - (self.x * rhs.x) - (self.y * rhs.y) - (self.z * rhs.z),
            x: (self.w * rhs.x) + (self.x * rhs.w) + (self.y * rhs.z) - (self.z * rhs.y),
            y: (self.w * rhs.y) - (self.x * rhs.z) + (self.y * rhs.w) + (self.z * rhs.x),
            z: (self.w * rhs.z) + (self.x * rhs.y) - (self.y * rhs.x) + (self.z * rhs.w),
        }
    }
}

impl Mul<f32> for Quat {
    type Output = Quat;

    fn mul(self, rhs: f32) -> Self {
        Self {
            w: self.w * rhs,
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl Mul<Quat> for f32 {
    type Output = Quat;

    fn mul(self, rhs: Quat) -> Quat {
        rhs * self
    }
}

impl Div<f32> for Quat {
    type Output = Quat;

    fn div(self, rhs: f32) -> Self {
        Self {
            w: self.w / rhs,
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl AbsDiffEq for Quat {
    type Epsilon = f32;

    fn default_epsilon() -> Self::Epsilon {
        f32::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        f32::abs_diff_eq(&self.w, &other.w, epsilon)
            && f32::abs_diff_eq(&self.x, &other.x, epsilon)
            && f32::abs_diff_eq(&self.y, &other.y, epsilon)
            && f32::abs_diff_eq(&self.z, &other.z, epsilon)
    }
}

impl RelativeEq for Quat {
    fn default_max_relative() -> Self::Epsilon {
        f32::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        f32::relative_eq(&self.w, &other.w, epsilon, max_relative)
            && f32::relative_eq(&self.x, &other.x, epsilon, max_relative)
            && f32::relative_eq(&self.y, &other.y, epsilon, max_relative)
            && f32::relative_eq(&self.z, &other.z, epsilon, max_relative)
    }
}

/// A Hamilton quaternion stored in `[w, x, y, z]` component order.
///
/// The type can represent general quaternions as well as rotations. Methods
/// whose names contain `rotate_unit` require `self` to be a unit quaternion;
/// use [`Quat::normalize`] when that invariant has not already been established.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    /// The scalar component.
    pub w: f32,

    /// The x component of the vector part.
    pub x: f32,

    /// The y component of the vector part.
    pub y: f32,

    /// The z component of the vector part.
    pub z: f32,
}

impl Quat {
    /// Creates a quaternion in `[w, x, y, z]` order without validation.
    ///
    /// This constructor permits non-unit quaternions, `NaN`, and infinity.
    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Quat { w, x, y, z }
    }

    /// Creates a quaternion with finite coefficients in `[w, x, y, z]` order.
    ///
    /// This method does not require or produce a unit quaternion. Call
    /// [`Quat::normalize`] when a unit quaternion is required.
    ///
    /// # Errors
    ///
    /// Returns [`QuatError::InvalidCoefficients`] if any coefficient is `NaN`
    /// or infinite.
    pub fn try_new(w: f32, x: f32, y: f32, z: f32) -> Result<Self, QuatError> {
        if !(w.is_finite() && x.is_finite() && y.is_finite() && z.is_finite()) {
            return Err(QuatError::InvalidCoefficients(w, x, y, z));
        }
        // if libm::sqrtf(w*w + x*x + y*y + z*z) != 1f32 {
        //     return Err(
        //         QuatError::InvalidCoefficients(w,x,y,z)
        //     );
        // }
        Ok(Quat { w, x, y, z })
    }

    /// Creates a quaternion from an array in `[w, x, y, z]` order.
    ///
    /// The array is stored without validation or normalization.
    pub fn from_array_wxyz(a: [f32; 4]) -> Self {
        Self {
            w: a[0],
            x: a[1],
            y: a[2],
            z: a[3],
        }
    }

    /// Returns the quaternion components by value in `[w, x, y, z]` order.
    pub fn as_array_wxyz(&self) -> [f32; 4] {
        [self.w, self.x, self.y, self.z]
    }

    /// Returns the multiplicative identity quaternion `[1, 0, 0, 0]`.
    ///
    /// As an attitude, this represents no rotation between frames.
    pub fn identity() -> Self {
        Self {
            w: 1f32,
            x: 0f32,
            y: 0f32,
            z: 0f32,
        }
    }

    /// Embeds a [`Vec3`] as the pure quaternion `[0, v.x, v.y, v.z]`.
    pub fn pure(v: Vec3) -> Self {
        Self {
            w: 0f32,
            x: v.x,
            y: v.y,
            z: v.z,
        }
    }

    /// Returns the quaternion's vector part `[x, y, z]` and discards `w`.
    pub fn vector_part(&self) -> Vec3 {
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    /// Returns the squared Euclidean norm of the four coefficients.
    pub fn norm_squared(&self) -> f32 {
        self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Returns the Euclidean norm of the four coefficients.
    pub fn norm(&self) -> f32 {
        libm::sqrtf(self.norm_squared())
    }

    /// Returns a normalized copy of this quaternion.
    ///
    /// This is an alias for [`Quat::normalize`].
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Quat::normalize`].
    pub fn normalized(self) -> Result<Self, QuatError> {
        self.normalize()
    }

    /// Returns a unit quaternion with the same direction as this quaternion.
    ///
    /// # Errors
    ///
    /// Returns [`QuatError::InvalidCoefficients`] if any coefficient, the norm,
    /// or a normalized coefficient is non-finite. Returns
    /// [`QuatError::NormTooSmall`] if the norm is at or below `1e-11`.
    pub fn normalize(self) -> Result<Self, QuatError> {
        if !(self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()) {
            return Err(QuatError::InvalidCoefficients(
                self.w, self.x, self.y, self.z,
            ));
        }

        let n: f32 = self.norm();
        if !n.is_finite() {
            return Err(QuatError::InvalidCoefficients(
                self.w, self.x, self.y, self.z,
            ));
        }
        if n <= 1e-11_f32 {
            return Err(QuatError::NormTooSmall(self));
        }

        let normalized = Self {
            w: self.w / n,
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
        };

        if !(normalized.w.is_finite()
            && normalized.x.is_finite()
            && normalized.y.is_finite()
            && normalized.z.is_finite())
        {
            return Err(QuatError::InvalidCoefficients(
                self.w, self.x, self.y, self.z,
            ));
        }

        Ok(normalized)
    }

    /// Normalizes this quaternion in place.
    ///
    /// The value is updated only when normalization succeeds.
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`Quat::normalize`].
    pub fn normalize_assign(&mut self) -> Result<(), QuatError> {
        *self = self.normalize()?;
        Ok(())
    }

    /// Returns the quaternion conjugate `[w, -x, -y, -z]`.
    ///
    /// For a unit quaternion, the conjugate is also its multiplicative inverse.
    pub fn conjugate(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// Rotates a vector from the current IMU/body frame (`I`) into the
    /// calibration/reference frame (`C`).
    ///
    /// This evaluates `q * [0, vector] * q.conjugate()`. `self` must be a unit
    /// quaternion for the result to be a pure rotation without scaling.
    pub fn rotate_unit_i_to_c(self, u: Vec3) -> Vec3 {
        (self * Quat::pure(u) * self.conjugate()).vector_part()
    }

    /// Rotates a vector from the calibration/reference frame (`C`) into the
    /// current IMU/body frame (`I`).
    ///
    /// This evaluates `q.conjugate() * [0, vector] * q`. `self` must be a unit
    /// quaternion for this to be the inverse of [`Quat::rotate_unit_i_to_c`].
    pub fn rotate_unit_c_to_i(self, v: Vec3) -> Vec3 {
        (self.conjugate() * Quat::pure(v) * self).vector_part()
    }

    /// Computes the attitude derivative from angular rate in body coordinates.
    ///
    /// This implements `q_dot = 0.5 * (q * [0, omega_i])`, where `omega_i` is
    /// expressed in the current IMU/body frame in radians per second. The
    /// returned value is a quaternion derivative, not a normalized attitude.
    pub fn derivative_from_body_rate(self, omega_i: Vec3) -> Self {
        0.5_f32 * (self * Quat::pure(omega_i))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use crate::quaternion::{Quat, QuatError};
    use crate::vec3::Vec3;

    const EPSILON: f32 = 1e-6_f32;

    fn assert_quat_relative_eq(actual: Quat, expected: Quat) {
        approx::assert_relative_eq!(actual.w, expected.w, epsilon = EPSILON);
        approx::assert_relative_eq!(actual.x, expected.x, epsilon = EPSILON);
        approx::assert_relative_eq!(actual.y, expected.y, epsilon = EPSILON);
        approx::assert_relative_eq!(actual.z, expected.z, epsilon = EPSILON);
    }

    fn assert_vec3_relative_eq(actual: Vec3, expected: Vec3) {
        approx::assert_relative_eq!(actual.x, expected.x, epsilon = EPSILON);
        approx::assert_relative_eq!(actual.y, expected.y, epsilon = EPSILON);
        approx::assert_relative_eq!(actual.z, expected.z, epsilon = EPSILON);
    }

    #[test]
    fn test_new_preserves_wxyz_component_order() {
        let q = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32);

        assert_eq!(q.w, 1.0_f32);
        assert_eq!(q.x, 2.0_f32);
        assert_eq!(q.y, 3.0_f32);
        assert_eq!(q.z, 4.0_f32);
    }

    #[test]
    fn test_new_allows_non_unit_quaternions() {
        let q = Quat::new(2.0_f32, -4.0_f32, 6.0_f32, -8.0_f32);

        assert_eq!(
            q,
            Quat {
                w: 2.0_f32,
                x: -4.0_f32,
                y: 6.0_f32,
                z: -8.0_f32,
            }
        );
    }

    #[test]
    fn test_try_new_rejects_invalid_coefficients() {
        assert!(matches!(
            Quat::try_new(f32::NAN, 0.0_f32, 0.0_f32, 0.0_f32),
            Err(QuatError::InvalidCoefficients(_, _, _, _))
        ));
        assert!(matches!(
            Quat::try_new(0.0_f32, f32::INFINITY, 0.0_f32, 0.0_f32),
            Err(QuatError::InvalidCoefficients(_, _, _, _))
        ));
        assert!(matches!(
            Quat::try_new(0.0_f32, 0.0_f32, f32::NEG_INFINITY, 0.0_f32),
            Err(QuatError::InvalidCoefficients(_, _, _, _))
        ));
    }

    #[test]
    fn test_try_new_accepts_finite_coefficients() {
        assert_eq!(
            Quat::try_new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32).unwrap(),
            Quat::new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32)
        );
    }

    #[test]
    fn test_identity() {
        assert_eq!(
            Quat::identity(),
            Quat {
                w: 1.0_f32,
                x: 0.0_f32,
                y: 0.0_f32,
                z: 0.0_f32,
            }
        );
    }

    #[test]
    fn test_pure() {
        let v = Vec3::new(1.0_f32, -2.0_f32, 3.0_f32);

        assert_eq!(
            Quat::pure(v),
            Quat {
                w: 0.0_f32,
                x: 1.0_f32,
                y: -2.0_f32,
                z: 3.0_f32,
            }
        );
    }

    #[test]
    fn test_from_array_wxyz() {
        assert_eq!(
            Quat::from_array_wxyz([1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32]),
            Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32)
        );
    }

    #[test]
    fn test_as_array_wxyz() {
        let q = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32);

        assert_eq!(q.as_array_wxyz(), [1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32]);
    }

    #[test]
    fn test_vector_part_returns_vec3() {
        let q = Quat::new(9.0_f32, 1.0_f32, -2.0_f32, 3.0_f32);

        assert_eq!(q.vector_part(), Vec3::new(1.0_f32, -2.0_f32, 3.0_f32));
    }

    #[test]
    fn test_norm_squared() {
        let q = Quat::new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32);

        assert_eq!(q.norm_squared(), 30.0_f32);
    }

    #[test]
    fn test_norm() {
        let q = Quat::new(1.0_f32, 2.0_f32, 2.0_f32, 4.0_f32);

        assert_eq!(q.norm(), 5.0_f32);
    }

    #[test]
    fn test_normalize_returns_unit_quaternion_without_mutating_original() {
        let q = Quat::new(2.0_f32, 0.0_f32, 0.0_f32, 0.0_f32);

        assert_eq!(q.normalize().unwrap(), Quat::identity());
        assert_eq!(q, Quat::new(2.0_f32, 0.0_f32, 0.0_f32, 0.0_f32));
    }

    #[test]
    fn test_normalize_preserves_direction() {
        let q = Quat::new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32);
        let normalized = q.normalize().unwrap();

        approx::assert_relative_eq!(normalized.norm(), 1.0_f32, epsilon = EPSILON);
        assert_quat_relative_eq(normalized, q / q.norm());
    }

    #[test]
    fn test_normalize_rejects_tiny_norm() {
        let q = Quat::new(1e-12_f32, 1e-12_f32, 1e-12_f32, 1e-12_f32);

        assert!(matches!(q.normalize(), Err(QuatError::NormTooSmall(_))));
    }

    #[test]
    fn test_normalize_rejects_zero_norm() {
        let q = Quat::new(0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32);

        assert!(matches!(q.normalize(), Err(QuatError::NormTooSmall(_))));
    }

    #[test]
    fn test_normalize_rejects_non_finite_coefficients_from_new() {
        assert!(matches!(
            Quat::new(f32::NAN, 0.0_f32, 0.0_f32, 0.0_f32).normalize(),
            Err(QuatError::InvalidCoefficients(_, _, _, _))
        ));
        assert!(matches!(
            Quat::new(0.0_f32, f32::INFINITY, 0.0_f32, 0.0_f32).normalize(),
            Err(QuatError::InvalidCoefficients(_, _, _, _))
        ));
    }

    #[test]
    fn test_normalize_rejects_non_finite_norm() {
        let q = Quat::new(f32::MAX, f32::MAX, f32::MAX, f32::MAX);

        assert!(matches!(
            q.normalize(),
            Err(QuatError::InvalidCoefficients(_, _, _, _))
        ));
    }

    #[test]
    fn test_normalize_assign_updates_in_place() {
        let mut q = Quat::new(0.0_f32, 3.0_f32, 4.0_f32, 0.0_f32);

        q.normalize_assign().unwrap();

        assert_quat_relative_eq(q, Quat::new(0.0_f32, 0.6_f32, 0.8_f32, 0.0_f32));
    }

    #[test]
    fn test_conjugate() {
        let q = Quat::new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32);

        assert_eq!(
            q.conjugate(),
            Quat::new(1.0_f32, 2.0_f32, -3.0_f32, 4.0_f32)
        );
    }

    #[test]
    fn test_unit_inverse_is_conjugate() {
        let q = Quat::new(0.5_f32, 0.5_f32, 0.5_f32, 0.5_f32);

        assert_quat_relative_eq(q * q.conjugate(), Quat::identity());
        assert_quat_relative_eq(q.conjugate() * q, Quat::identity());
    }

    #[test]
    fn test_add() {
        let q1 = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32);
        let q2 = Quat::new(0.5_f32, -1.0_f32, 2.0_f32, -3.0_f32);

        assert_eq!(q1 + q2, Quat::new(1.5_f32, 1.0_f32, 5.0_f32, 1.0_f32));
    }

    #[test]
    fn test_add_assign() {
        let mut q = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32);

        q += Quat::new(0.5_f32, -1.0_f32, 2.0_f32, -3.0_f32);

        assert_eq!(q, Quat::new(1.5_f32, 1.0_f32, 5.0_f32, 1.0_f32));
    }

    #[test]
    fn test_sub() {
        let q1 = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32);
        let q2 = Quat::new(0.5_f32, -1.0_f32, 2.0_f32, -3.0_f32);

        assert_eq!(q1 - q2, Quat::new(0.5_f32, 3.0_f32, 1.0_f32, 7.0_f32));
    }

    #[test]
    fn test_sub_assign() {
        let mut q = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32);

        q -= Quat::new(0.5_f32, -1.0_f32, 2.0_f32, -3.0_f32);

        assert_eq!(q, Quat::new(0.5_f32, 3.0_f32, 1.0_f32, 7.0_f32));
    }

    #[test]
    fn test_scalar_mul() {
        let q = Quat::new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32);

        assert_eq!(q * 2.0_f32, Quat::new(2.0_f32, -4.0_f32, 6.0_f32, -8.0_f32));
        assert_eq!(2.0_f32 * q, Quat::new(2.0_f32, -4.0_f32, 6.0_f32, -8.0_f32));
    }

    #[test]
    fn test_scalar_div() {
        let q = Quat::new(2.0_f32, -4.0_f32, 6.0_f32, -8.0_f32);

        assert_eq!(q / 2.0_f32, Quat::new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32));
    }

    #[test]
    fn test_hamilton_product_identity() {
        let q = Quat::new(1.0_f32, -2.0_f32, 3.0_f32, -4.0_f32);

        assert_eq!(Quat::identity() * q, q);
        assert_eq!(q * Quat::identity(), q);
    }

    #[test]
    fn test_hamilton_product_basis_rules() {
        let one = Quat::identity();
        let i = Quat::new(0.0_f32, 1.0_f32, 0.0_f32, 0.0_f32);
        let j = Quat::new(0.0_f32, 0.0_f32, 1.0_f32, 0.0_f32);
        let k = Quat::new(0.0_f32, 0.0_f32, 0.0_f32, 1.0_f32);

        assert_eq!(i * i, -1.0_f32 * one);
        assert_eq!(j * j, -1.0_f32 * one);
        assert_eq!(k * k, -1.0_f32 * one);
        assert_eq!(i * j, k);
        assert_eq!(j * k, i);
        assert_eq!(k * i, j);
        assert_eq!(j * i, -1.0_f32 * k);
        assert_eq!(k * j, -1.0_f32 * i);
        assert_eq!(i * k, -1.0_f32 * j);
    }

    #[test]
    fn test_hamilton_product_general_case() {
        let p = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32);
        let q = Quat::new(5.0_f32, 6.0_f32, 7.0_f32, 8.0_f32);

        assert_eq!(p * q, Quat::new(-60.0_f32, 12.0_f32, 30.0_f32, 24.0_f32));
        assert_eq!(q * p, Quat::new(-60.0_f32, 20.0_f32, 14.0_f32, 32.0_f32));
    }

    #[test]
    fn test_hamilton_product_is_not_commutative() {
        let i = Quat::new(0.0_f32, 1.0_f32, 0.0_f32, 0.0_f32);
        let j = Quat::new(0.0_f32, 0.0_f32, 1.0_f32, 0.0_f32);

        assert_ne!(i * j, j * i);
    }

    #[test]
    fn test_rotate_unit_i_to_c_identity() {
        let v = Vec3::new(1.0_f32, -2.0_f32, 3.0_f32);

        assert_eq!(Quat::identity().rotate_unit_i_to_c(v), v);
    }

    #[test]
    fn test_rotate_unit_c_to_i_identity() {
        let v = Vec3::new(1.0_f32, -2.0_f32, 3.0_f32);

        assert_eq!(Quat::identity().rotate_unit_c_to_i(v), v);
    }

    #[test]
    fn test_rotate_unit_i_to_c_90_degrees_about_z() {
        let half_angle = core::f32::consts::FRAC_PI_4;
        let q = Quat::new(
            libm::cosf(half_angle),
            0.0_f32,
            0.0_f32,
            libm::sinf(half_angle),
        );

        assert_vec3_relative_eq(
            q.rotate_unit_i_to_c(Vec3::new(1.0_f32, 0.0_f32, 0.0_f32)),
            Vec3::new(0.0_f32, 1.0_f32, 0.0_f32),
        );
    }

    #[test]
    fn test_rotate_unit_c_to_i_uses_conjugate_direction() {
        let half_angle = core::f32::consts::FRAC_PI_4;
        let q = Quat::new(
            libm::cosf(half_angle),
            0.0_f32,
            0.0_f32,
            libm::sinf(half_angle),
        );

        assert_vec3_relative_eq(
            q.rotate_unit_c_to_i(Vec3::new(0.0_f32, 1.0_f32, 0.0_f32)),
            Vec3::new(1.0_f32, 0.0_f32, 0.0_f32),
        );
    }

    #[test]
    fn test_rotate_round_trip() {
        let half_angle = core::f32::consts::FRAC_PI_4;
        let q = Quat::new(
            libm::cosf(half_angle),
            0.0_f32,
            libm::sinf(half_angle),
            0.0_f32,
        );
        let v = Vec3::new(1.0_f32, 2.0_f32, 3.0_f32);

        assert_vec3_relative_eq(q.rotate_unit_c_to_i(q.rotate_unit_i_to_c(v)), v);
    }

    #[test]
    fn test_derivative_from_body_rate_matches_schematic_order() {
        let q_ci = Quat::new(0.5_f32, 0.5_f32, 0.5_f32, 0.5_f32);
        let omega_i = Vec3::new(1.0_f32, 2.0_f32, 3.0_f32);

        assert_quat_relative_eq(
            q_ci.derivative_from_body_rate(omega_i),
            0.5_f32 * (q_ci * Quat::pure(omega_i)),
        );
    }

    #[test]
    fn test_derivative_from_body_rate_is_not_world_rate_order() {
        let q_ci = Quat::new(0.5_f32, 0.5_f32, 0.5_f32, 0.5_f32);
        let omega_i = Vec3::new(1.0_f32, 2.0_f32, 3.0_f32);

        assert_ne!(
            q_ci.derivative_from_body_rate(omega_i),
            0.5_f32 * (Quat::pure(omega_i) * q_ci)
        );
    }

    #[test]
    fn test_display_invalid_coefficients_error() {
        let err = QuatError::InvalidCoefficients(1.0_f32, f32::NAN, 2.0_f32, 3.0_f32);
        let message = std::format!("{}", err);

        assert!(message.contains("candidate coefficients"));
        assert!(message.contains("w:1"));
        assert!(message.contains("x:NaN"));
    }

    #[test]
    fn test_display_norm_too_small_error() {
        let err = QuatError::NormTooSmall(Quat::new(1e-12_f32, 0.0_f32, 0.0_f32, 0.0_f32));
        let message = std::format!("{}", err);

        assert!(message.contains("quaternion"));
        assert!(message.contains("too small"));
    }
}
