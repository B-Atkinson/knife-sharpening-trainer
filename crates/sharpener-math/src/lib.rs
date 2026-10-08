//! Small, `no_std` mathematical primitives for the knife-sharpening estimator.
//!
//! The crate provides:
//!
//! - [`vec3::Vec3`] for three-dimensional vector operations.
//! - [`quaternion::Quat`] for Hamilton quaternions in `[w, x, y, z]` order.
//! - [`mat3::Mat3`] for general three-by-three matrices and frame rotations.
//!
//! Orientation APIs use `I` for the current IMU/body frame and `C` for the
//! calibration/reference frame. Unless a checked constructor or normalization
//! method says otherwise, arithmetic is intentionally unchecked and follows
//! normal `f32` behavior for `NaN`, infinity, overflow, and division by zero.
//!
//! # Example
//!
//! ```
//! use sharpener_math::mat3::Mat3;
//! use sharpener_math::quaternion::Quat;
//! use sharpener_math::vec3::Vec3;
//!
//! let attitude_i_to_c = Quat::identity();
//! let rotation = Mat3::try_from_quat_i_to_c(attitude_i_to_c).unwrap();
//! let body_vector = Vec3::new(1.0, 0.0, 0.0);
//!
//! assert_eq!(rotation.rotate_unit_i_to_c(body_vector), body_vector);
//! ```

#![no_std]

pub mod mat3;
pub mod quaternion;
pub mod vec3;
