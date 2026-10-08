//! Three-by-three matrices used for coordinate-frame rotations.
//!
//! [`Mat3`] stores coefficients in row-major order and can represent any
//! three-by-three matrix. A matrix created by [`Mat3::try_from_quat_i_to_c`]
//! is a rotation matrix that transforms vectors from the current IMU/body
//! frame (`I`) into the calibration/reference frame (`C`).
//!
//! # Example
//!
//! ```
//! use sharpener_math::mat3::Mat3;
//! use sharpener_math::vec3::Vec3;
//!
//! let matrix = Mat3::identity();
//! let vector = Vec3::new(1.0, 2.0, 3.0);
//!
//! assert_eq!(matrix * vector, vector);
//! ```

use crate::quaternion::{Quat, QuatError};
use crate::vec3::Vec3;
use approx::{AbsDiffEq, RelativeEq};
use core::fmt::Formatter;
use core::ops::Mul;

/// An error encountered while constructing a validated [`Mat3`].
#[derive(Debug)]
pub enum Mat3Error {
    /// A general matrix contained at least one `NaN` or infinite coefficient.
    InvalidCoefficients([[f32; 3]; 3]),

    /// A quaternion could not be normalized before conversion to a matrix.
    InvalidQuaternion(QuatError),
}

impl core::fmt::Display for Mat3Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Mat3Error::InvalidCoefficients(rows) => {
                write!(f, "matrix coefficients {rows:?} must be finite")
            }
            Mat3Error::InvalidQuaternion(e) => write!(f, "quaternion operation error: {}", e),
        }
    }
}

impl From<QuatError> for Mat3Error {
    fn from(error: QuatError) -> Self {
        Mat3Error::InvalidQuaternion(error)
    }
}

impl AbsDiffEq for Mat3 {
    type Epsilon = f32;

    fn default_epsilon() -> Self::Epsilon {
        f32::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        let mut result = true;
        for (row, row_array) in self.0.iter().enumerate().take(3) {
            for (col, _value) in row_array.iter().enumerate().take(3) {
                result = result && f32::abs_diff_eq(_value, &other.0[row][col], epsilon);
            }
        }
        result
    }
}

impl RelativeEq for Mat3 {
    fn default_max_relative() -> Self::Epsilon {
        f32::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        let mut result = true;
        for (row, row_array) in self.0.iter().enumerate().take(3) {
            for (col, _value) in row_array.iter().enumerate().take(3) {
                result =
                    result && f32::relative_eq(_value, &other.0[row][col], epsilon, max_relative);
            }
        }
        result
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        let x: f32 = self.0[0][0] * rhs.x + self.0[0][1] * rhs.y + self.0[0][2] * rhs.z;
        let y: f32 = self.0[1][0] * rhs.x + self.0[1][1] * rhs.y + self.0[1][2] * rhs.z;
        let z: f32 = self.0[2][0] * rhs.x + self.0[2][1] * rhs.y + self.0[2][2] * rhs.z;

        Vec3::new(x, y, z)
    }
}

/// A general three-by-three matrix stored in row-major order.
///
/// Element `self.0[row][column]` is the coefficient at the specified row and
/// column. The type does not guarantee that a matrix is finite, orthogonal, or
/// invertible; those properties depend on how it was constructed.
///
/// Use [`Mat3::try_new`] when finite coefficients are required. Use
/// [`Mat3::try_from_quat_i_to_c`] to construct a rotation matrix from a
/// quaternion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat3(
    /// The matrix coefficients in row-major order.
    pub [[f32; 3]; 3],
);

impl Mat3 {
    /// Creates a general matrix from row-major coefficients without validation.
    ///
    /// This constructor permits `NaN` and infinite coefficients. Use
    /// [`Mat3::try_new`] when non-finite values should be rejected.
    pub fn new(rows: [[f32; 3]; 3]) -> Self {
        Self(rows)
    }

    /// Creates a general matrix from finite row-major coefficients.
    ///
    /// This validates only that every coefficient is finite. It does not
    /// require the matrix to be orthogonal, invertible, or a rotation matrix.
    ///
    /// # Errors
    ///
    /// Returns [`Mat3Error::InvalidCoefficients`] if any coefficient is `NaN`
    /// or infinite.
    pub fn try_new(rows: [[f32; 3]; 3]) -> Result<Self, Mat3Error> {
        if rows.iter().flatten().any(|value| !value.is_finite()) {
            return Err(Mat3Error::InvalidCoefficients(rows));
        }

        Ok(Self(rows))
    }

    /// Returns the identity matrix.
    ///
    /// Multiplying any [`Vec3`] by this matrix leaves the vector unchanged.
    pub fn identity() -> Self {
        Mat3([[1f32, 0f32, 0f32], [0f32, 1f32, 0f32], [0f32, 0f32, 1f32]])
    }

    /// Returns a matrix whose nine coefficients are zero.
    pub fn zeros() -> Self {
        Mat3([[0f32, 0f32, 0f32], [0f32, 0f32, 0f32], [0f32, 0f32, 0f32]])
    }

    /// Returns a matrix whose nine coefficients are one.
    pub fn ones() -> Self {
        Mat3([[1f32, 1f32, 1f32], [1f32, 1f32, 1f32], [1f32, 1f32, 1f32]])
    }

    /// Returns this matrix with its rows and columns exchanged.
    ///
    /// For a rotation matrix, the transpose is also its inverse. For a general
    /// matrix, transposition does not necessarily produce an inverse.
    pub fn transpose(self) -> Self {
        let mut new_mat: Self = self;
        for row in 1..3usize {
            for col in 0..row {
                new_mat.0[col][row] = self.0[row][col];
                new_mat.0[row][col] = self.0[col][row];
            }
        }
        new_mat
    }

    /// Constructs the `I -> C` rotation matrix represented by a quaternion.
    ///
    /// The input quaternion uses `[w, x, y, z]` component order and is
    /// normalized before conversion. The returned row-major matrix represents
    /// the same transformation as calling [`Quat::rotate_unit_i_to_c`] with
    /// the normalized quaternion.
    ///
    /// # Errors
    ///
    /// Returns [`Mat3Error::InvalidQuaternion`] when quaternion normalization
    /// fails because its coefficients or norm are non-finite, or because its
    /// norm is too small.
    pub fn try_from_quat_i_to_c(q: Quat) -> Result<Self, Mat3Error> {
        let q_norm: Quat = q.normalize()?;
        Ok(Mat3([
            [
                1f32 - 2f32 * (q_norm.y * q_norm.y + q_norm.z * q_norm.z),
                2f32 * (q_norm.x * q_norm.y - q_norm.w * q_norm.z),
                2f32 * (q_norm.x * q_norm.z + q_norm.w * q_norm.y),
            ],
            [
                2f32 * (q_norm.x * q_norm.y + q_norm.w * q_norm.z),
                1f32 - 2f32 * (q_norm.x * q_norm.x + q_norm.z * q_norm.z),
                2f32 * (q_norm.y * q_norm.z - q_norm.w * q_norm.x),
            ],
            [
                2f32 * (q_norm.x * q_norm.z - q_norm.w * q_norm.y),
                2f32 * (q_norm.y * q_norm.z + q_norm.w * q_norm.x),
                1f32 - 2f32 * (q_norm.x * q_norm.x + q_norm.y * q_norm.y),
            ],
        ]))
    }

    /// Transforms a vector from the current IMU/body frame (`I`) into the
    /// calibration/reference frame (`C`).
    ///
    /// This is equivalent to `self * vector_i`. For the operation to represent
    /// a pure rotation, `self` must be a valid rotation matrix.
    pub fn rotate_unit_i_to_c(self, vector_i: Vec3) -> Vec3 {
        self * vector_i
    }

    /// Transforms a vector from the calibration/reference frame (`C`) into the
    /// current IMU/body frame (`I`).
    ///
    /// This is equivalent to `self.transpose() * vector_c`. It is the inverse
    /// of [`Mat3::rotate_unit_i_to_c`] only when `self` is an orthogonal rotation
    /// matrix, such as one returned by [`Mat3::try_from_quat_i_to_c`].
    pub fn rotate_unit_c_to_i(self, vector_c: Vec3) -> Vec3 {
        self.transpose() * vector_c
    }

    /// Returns the matrix coefficients by value in row-major order.
    pub fn as_array_rows(&self) -> [[f32; 3]; 3] {
        self.0
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::{Mat3, Mat3Error};
    use crate::quaternion::{Quat, QuatError};
    use crate::vec3::Vec3;
    use approx::{AbsDiffEq, RelativeEq};

    const EPSILON: f32 = 1e-6_f32;

    #[test]
    fn test_new_and_as_array_rows_preserve_row_major_coefficients() {
        let rows = [
            [1.0_f32, 2.0_f32, 3.0_f32],
            [4.0_f32, 5.0_f32, 6.0_f32],
            [7.0_f32, 8.0_f32, 9.0_f32],
        ];

        assert_eq!(Mat3::new(rows).as_array_rows(), rows);
    }

    #[test]
    fn test_try_new_accepts_finite_general_matrix() {
        let rows = [
            [1.0_f32, -2.0_f32, 3.5_f32],
            [4.25_f32, 0.0_f32, -6.0_f32],
            [7.0_f32, 8.0_f32, 9.0_f32],
        ];

        assert_eq!(Mat3::try_new(rows).unwrap(), Mat3::new(rows));
    }

    #[test]
    fn test_try_new_rejects_non_finite_coefficient_in_any_position() {
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            for row in 0..3 {
                for column in 0..3 {
                    let mut rows = Mat3::identity().as_array_rows();
                    rows[row][column] = invalid;

                    assert!(matches!(
                        Mat3::try_new(rows),
                        Err(Mat3Error::InvalidCoefficients(_))
                    ));
                }
            }
        }
    }

    #[test]
    fn test_identity_multiplication_preserves_vector() {
        let vector = Vec3::new(1.0_f32, -2.0_f32, 3.0_f32);

        assert_eq!(Mat3::identity() * vector, vector);
    }

    #[test]
    fn test_matrix_vector_multiplication_uses_row_major_coefficients() {
        let matrix = Mat3::new([
            [1.0_f32, 2.0_f32, 3.0_f32],
            [4.0_f32, 5.0_f32, 6.0_f32],
            [7.0_f32, 8.0_f32, 9.0_f32],
        ]);
        let vector = Vec3::new(2.0_f32, -1.0_f32, 0.5_f32);

        assert_eq!(matrix * vector, Vec3::new(1.5_f32, 6.0_f32, 10.5_f32));
    }

    #[test]
    fn test_transpose_swaps_rows_and_columns() {
        let matrix = Mat3::new([
            [1.0_f32, 2.0_f32, 3.0_f32],
            [4.0_f32, 5.0_f32, 6.0_f32],
            [7.0_f32, 8.0_f32, 9.0_f32],
        ]);

        assert_eq!(
            matrix.transpose(),
            Mat3::new([
                [1.0_f32, 4.0_f32, 7.0_f32],
                [2.0_f32, 5.0_f32, 8.0_f32],
                [3.0_f32, 6.0_f32, 9.0_f32],
            ])
        );
        assert_eq!(matrix.transpose().transpose(), matrix);
    }

    #[test]
    fn test_approximate_comparisons_check_all_coefficients() {
        let matrix = Mat3::identity();
        let close = Mat3::new([
            [1.0_f32 + 0.5e-6_f32, 0.0_f32, 0.0_f32],
            [0.0_f32, 1.0_f32, 0.0_f32],
            [0.0_f32, 0.0_f32, 1.0_f32 - 0.5e-6_f32],
        ]);
        let different = Mat3::new([
            [1.0_f32, 0.0_f32, 0.0_f32],
            [0.0_f32, 1.0_f32, 0.0_f32],
            [0.0_f32, 0.0_f32, 0.9_f32],
        ]);

        assert!(matrix.abs_diff_eq(&close, EPSILON));
        assert!(matrix.relative_eq(&close, EPSILON, EPSILON));
        assert!(!matrix.abs_diff_eq(&different, EPSILON));
        assert!(!matrix.relative_eq(&different, EPSILON, EPSILON));
    }

    #[test]
    fn test_quaternion_identity_produces_identity_matrix() {
        let matrix = Mat3::try_from_quat_i_to_c(Quat::identity()).unwrap();

        assert!(matrix.abs_diff_eq(&Mat3::identity(), EPSILON));
    }

    #[test]
    fn test_quaternion_constructor_rejects_invalid_quaternion() {
        assert!(matches!(
            Mat3::try_from_quat_i_to_c(Quat::new(f32::NAN, 0.0_f32, 0.0_f32, 0.0_f32)),
            Err(Mat3Error::InvalidQuaternion(
                QuatError::InvalidCoefficients(_, _, _, _)
            ))
        ));
        assert!(matches!(
            Mat3::try_from_quat_i_to_c(Quat::new(0.0_f32, 0.0_f32, 0.0_f32, 0.0_f32)),
            Err(Mat3Error::InvalidQuaternion(QuatError::NormTooSmall(_)))
        ));
    }

    #[test]
    fn test_non_unit_quaternion_is_normalized_before_conversion() {
        let half_angle = core::f32::consts::FRAC_PI_4;
        let scaled_quaternion = Quat::new(
            2.0_f32 * libm::cosf(half_angle),
            0.0_f32,
            0.0_f32,
            2.0_f32 * libm::sinf(half_angle),
        );
        let matrix = Mat3::try_from_quat_i_to_c(scaled_quaternion).unwrap();

        assert!(matrix.relative_eq(
            &Mat3::new([
                [0.0_f32, -1.0_f32, 0.0_f32],
                [1.0_f32, 0.0_f32, 0.0_f32],
                [0.0_f32, 0.0_f32, 1.0_f32],
            ]),
            EPSILON,
            EPSILON,
        ));
    }

    #[test]
    fn test_matrix_and_quaternion_rotations_are_equivalent() {
        let quaternion = Quat::new(1.0_f32, 2.0_f32, 3.0_f32, 4.0_f32)
            .normalize()
            .unwrap();
        let matrix = Mat3::try_from_quat_i_to_c(quaternion).unwrap();
        let vector = Vec3::new(2.0_f32, -1.0_f32, 0.5_f32);

        assert!(matrix.rotate_unit_i_to_c(vector).relative_eq(
            &quaternion.rotate_unit_i_to_c(vector),
            EPSILON,
            EPSILON
        ));
        assert!(matrix.rotate_unit_c_to_i(vector).relative_eq(
            &quaternion.rotate_unit_c_to_i(vector),
            EPSILON,
            EPSILON
        ));
    }

    #[test]
    fn test_rotation_round_trip_uses_transpose_direction() {
        let half_angle = core::f32::consts::FRAC_PI_4;
        let quaternion = Quat::new(
            libm::cosf(half_angle),
            0.0_f32,
            0.0_f32,
            libm::sinf(half_angle),
        );
        let matrix = Mat3::try_from_quat_i_to_c(quaternion).unwrap();
        let vector_i = Vec3::new(1.0_f32, 2.0_f32, 3.0_f32);
        let vector_c = matrix.rotate_unit_i_to_c(vector_i);

        assert!(vector_c.relative_eq(&Vec3::new(-2.0_f32, 1.0_f32, 3.0_f32), EPSILON, EPSILON,));
        assert!(
            matrix
                .rotate_unit_c_to_i(vector_c)
                .relative_eq(&vector_i, EPSILON, EPSILON)
        );
    }

    #[test]
    fn test_error_display_includes_context() {
        let rows = [
            [f32::NAN, 0.0_f32, 0.0_f32],
            [0.0_f32, 1.0_f32, 0.0_f32],
            [0.0_f32, 0.0_f32, 1.0_f32],
        ];
        let message = std::format!("{}", Mat3Error::InvalidCoefficients(rows));

        assert!(message.contains("matrix coefficients"));
        assert!(message.contains("NaN"));
        assert!(message.contains("must be finite"));
    }
}
