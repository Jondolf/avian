//! This mod re-exports the correct versions of floating-point operations with
//! unspecified precision in the standard library depending on whether the `libm`
//! crate feature is enabled.
//!
//! All the functions here are named according to their versions in the standard
//! library.
//!
//! It also provides `no_std` compatible alternatives to certain floating-point
//! operations which are not provided in the [`core`] library.

// Note: There are some Rust methods with unspecified precision without a `libm`
// equivalent:
// - `f64::powi` (integer powers)
// - `f64::log` (logarithm with specified base)
// - `f64::abs_sub` (actually unsure if `libm` has this, but don't use it regardless)
//
// Additionally, the following nightly API functions are not presently integrated
// into this, but they would be candidates once standardized:
// - `f64::gamma`
// - `f64::ln_gamma`

#[cfg(not(feature = "libm"))]
#[expect(
    clippy::disallowed_methods,
    reason = "Many of the disallowed methods are disallowed to force code to use the feature-conditional re-exports from this module, but this module itself is exempt from that rule."
)]
mod std_ops {

    /// Raises a number to a floating point power.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn powf(x: f64, y: f64) -> f64 {
        f64::powf(x, y)
    }

    /// Returns `e^(self)`, (the exponential function).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn exp(x: f64) -> f64 {
        f64::exp(x)
    }

    /// Returns `2^(self)`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn exp2(x: f64) -> f64 {
        f64::exp2(x)
    }

    /// Returns the natural logarithm of the number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn ln(x: f64) -> f64 {
        f64::ln(x)
    }

    /// Returns the base 2 logarithm of the number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn log2(x: f64) -> f64 {
        f64::log2(x)
    }

    /// Returns the base 10 logarithm of the number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn log10(x: f64) -> f64 {
        f64::log10(x)
    }

    /// Returns the cube root of a number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn cbrt(x: f64) -> f64 {
        f64::cbrt(x)
    }

    /// Compute the distance between the origin and a point `(x, y)` on the Euclidean plane.
    /// Equivalently, compute the length of the hypotenuse of a right-angle triangle with other sides having length `x.abs()` and `y.abs()`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn hypot(x: f64, y: f64) -> f64 {
        f64::hypot(x, y)
    }

    /// Computes the sine of a number (in radians).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn sin(x: f64) -> f64 {
        f64::sin(x)
    }

    /// Computes the cosine of a number (in radians).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn cos(x: f64) -> f64 {
        f64::cos(x)
    }

    /// Computes the tangent of a number (in radians).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn tan(x: f64) -> f64 {
        f64::tan(x)
    }

    /// Computes the arcsine of a number. Return value is in radians in
    /// the range [-pi/2, pi/2] or NaN if the number is outside the range
    /// [-1, 1].
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn asin(x: f64) -> f64 {
        f64::asin(x)
    }

    /// Computes the arccosine of a number. Return value is in radians in
    /// the range [0, pi] or NaN if the number is outside the range
    /// [-1, 1].
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn acos(x: f64) -> f64 {
        f64::acos(x)
    }

    /// Computes the arctangent of a number. Return value is in radians in the
    /// range [-pi/2, pi/2];
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn atan(x: f64) -> f64 {
        f64::atan(x)
    }

    /// Computes the four-quadrant arctangent of `y` and `x` in radians.
    ///
    /// * `x = 0`, `y = 0`: `0`
    /// * `x >= 0`: `arctan(y/x)` -> `[-pi/2, pi/2]`
    /// * `y >= 0`: `arctan(y/x) + pi` -> `(pi/2, pi]`
    /// * `y < 0`: `arctan(y/x) - pi` -> `(-pi, -pi/2)`
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn atan2(y: f64, x: f64) -> f64 {
        f64::atan2(y, x)
    }

    /// Simultaneously computes the sine and cosine of the number, `x`. Returns
    /// `(sin(x), cos(x))`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn sin_cos(x: f64) -> (f64, f64) {
        f64::sin_cos(x)
    }

    /// Returns `e^(self) - 1` in a way that is accurate even if the
    /// number is close to zero.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn exp_m1(x: f64) -> f64 {
        f64::exp_m1(x)
    }

    /// Returns `ln(1+n)` (natural logarithm) more accurately than if
    /// the operations were performed separately.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn ln_1p(x: f64) -> f64 {
        f64::ln_1p(x)
    }

    /// Hyperbolic sine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn sinh(x: f64) -> f64 {
        f64::sinh(x)
    }

    /// Hyperbolic cosine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn cosh(x: f64) -> f64 {
        f64::cosh(x)
    }

    /// Hyperbolic tangent function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn tanh(x: f64) -> f64 {
        f64::tanh(x)
    }

    /// Inverse hyperbolic sine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn asinh(x: f64) -> f64 {
        f64::asinh(x)
    }

    /// Inverse hyperbolic cosine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn acosh(x: f64) -> f64 {
        f64::acosh(x)
    }

    /// Inverse hyperbolic tangent function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn atanh(x: f64) -> f64 {
        f64::atanh(x)
    }
}

#[cfg(feature = "libm")]
mod libm_ops {

    /// Raises a number to a floating point power.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn powf(x: f64, y: f64) -> f64 {
        libm::pow(x, y)
    }

    /// Returns `e^(self)`, (the exponential function).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn exp(x: f64) -> f64 {
        libm::exp(x)
    }

    /// Returns `2^(self)`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn exp2(x: f64) -> f64 {
        libm::exp2(x)
    }

    /// Returns the natural logarithm of the number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn ln(x: f64) -> f64 {
        // This isn't documented in `libm` but this is actually the base e logarithm.
        libm::log(x)
    }

    /// Returns the base 2 logarithm of the number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn log2(x: f64) -> f64 {
        libm::log2(x)
    }

    /// Returns the base 10 logarithm of the number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn log10(x: f64) -> f64 {
        libm::log10(x)
    }

    /// Returns the cube root of a number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn cbrt(x: f64) -> f64 {
        libm::cbrt(x)
    }

    /// Compute the distance between the origin and a point `(x, y)` on the Euclidean plane.
    ///
    /// Equivalently, compute the length of the hypotenuse of a right-angle triangle with other sides having length `x.abs()` and `y.abs()`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn hypot(x: f64, y: f64) -> f64 {
        libm::hypot(x, y)
    }

    /// Computes the sine of a number (in radians).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn sin(x: f64) -> f64 {
        libm::sin(x)
    }

    /// Computes the cosine of a number (in radians).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn cos(x: f64) -> f64 {
        libm::cos(x)
    }

    /// Computes the tangent of a number (in radians).
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn tan(x: f64) -> f64 {
        libm::tan(x)
    }

    /// Computes the arcsine of a number. Return value is in radians in
    /// the range [-pi/2, pi/2] or NaN if the number is outside the range
    /// [-1, 1].
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn asin(x: f64) -> f64 {
        libm::asin(x)
    }

    /// Computes the arccosine of a number. Return value is in radians in
    /// Hyperbolic tangent function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    /// the range [0, pi] or NaN if the number is outside the range
    /// [-1, 1].
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn acos(x: f64) -> f64 {
        libm::acos(x)
    }

    /// Computes the arctangent of a number. Return value is in radians in the
    /// range [-pi/2, pi/2];
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn atan(x: f64) -> f64 {
        libm::atan(x)
    }

    /// Computes the four-quadrant arctangent of `y` and `x` in radians.
    ///
    /// * `x = 0`, `y = 0`: `0`
    /// * `x >= 0`: `arctan(y/x)` -> `[-pi/2, pi/2]`
    /// * `y >= 0`: `arctan(y/x) + pi` -> `(pi/2, pi]`
    /// * `y < 0`: `arctan(y/x) - pi` -> `(-pi, -pi/2)`
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn atan2(y: f64, x: f64) -> f64 {
        libm::atan2(y, x)
    }

    /// Simultaneously computes the sine and cosine of the number, `x`. Returns
    /// `(sin(x), cos(x))`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn sin_cos(x: f64) -> (f64, f64) {
        libm::sincos(x)
    }

    /// Returns `e^(self) - 1` in a way that is accurate even if the
    /// number is close to zero.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn exp_m1(x: f64) -> f64 {
        libm::expm1(x)
    }

    /// Returns `ln(1+n)` (natural logarithm) more accurately than if
    /// the operations were performed separately.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn ln_1p(x: f64) -> f64 {
        libm::log1p(x)
    }

    /// Hyperbolic sine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn sinh(x: f64) -> f64 {
        libm::sinh(x)
    }

    /// Hyperbolic cosine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn cosh(x: f64) -> f64 {
        libm::cosh(x)
    }

    /// Hyperbolic tangent function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn tanh(x: f64) -> f64 {
        libm::tanh(x)
    }

    /// Inverse hyperbolic sine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn asinh(x: f64) -> f64 {
        libm::asinh(x)
    }

    /// Inverse hyperbolic cosine function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn acosh(x: f64) -> f64 {
        libm::acosh(x)
    }

    /// Inverse hyperbolic tangent function.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn atanh(x: f64) -> f64 {
        libm::atanh(x)
    }
}

#[cfg(all(feature = "libm", not(feature = "std")))]
mod libm_ops_for_no_std {
    //! Provides standardized names for [`f64`] operations which may not be
    //! supported on `no_std` platforms.
    //! On `no_std` platforms, this forwards to the implementations provided
    //! by [`libm`].

    /// Calculates the least nonnegative remainder of `self (mod rhs)`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn rem_euclid(x: f64, y: f64) -> f64 {
        let result = libm::remainder(x, y);

        // libm::remainderf has a range of -y/2 to +y/2
        if result < 0. {
            result + y
        } else {
            result
        }
    }

    /// Computes the absolute value of x.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn abs(x: f64) -> f64 {
        libm::fabs(x)
    }

    /// Returns the square root of a number.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn sqrt(x: f64) -> f64 {
        libm::sqrt(x)
    }

    /// Returns a number composed of the magnitude of `x` and the sign of `y`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn copysign(x: f64, y: f64) -> f64 {
        libm::copysign(x, y)
    }

    /// Returns the nearest integer to `x`. If a value is half-way between two integers, round away from `0.0`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn round(x: f64) -> f64 {
        libm::round(x)
    }

    /// Returns the largest integer less than or equal to `x`.
    ///
    /// Precision is specified when the `libm` feature is enabled.
    #[inline(always)]
    pub fn floor(x: f64) -> f64 {
        libm::floor(x)
    }

    /// Returns the fractional part of `x`.
    ///
    /// This function always returns the precise result.
    #[inline(always)]
    pub fn fract(x: f64) -> f64 {
        libm::modf(x).0
    }
}

#[cfg(feature = "std")]
#[expect(
    clippy::disallowed_methods,
    reason = "Many of the disallowed methods are disallowed to force code to use the feature-conditional re-exports from this module, but this module itself is exempt from that rule."
)]
mod std_ops_for_no_std {
    //! Provides standardized names for [`f64`] operations which may not be
    //! supported on `no_std` platforms.
    //! On `std` platforms, this forwards directly to the implementations provided
    //! by [`std`].

    /// Calculates the least nonnegative remainder of `x (mod y)`.
    ///
    /// The result of this operation is guaranteed to be the rounded infinite-precision result.
    #[inline(always)]
    pub fn rem_euclid(x: f64, y: f64) -> f64 {
        f64::rem_euclid(x, y)
    }

    /// Computes the absolute value of x.
    ///
    /// This function always returns the precise result.
    #[inline(always)]
    pub fn abs(x: f64) -> f64 {
        f64::abs(x)
    }

    /// Returns the square root of a number.
    ///
    /// The result of this operation is guaranteed to be the rounded infinite-precision result.
    /// It is specified by IEEE 754 as `squareRoot` and guaranteed not to change.
    #[inline(always)]
    pub fn sqrt(x: f64) -> f64 {
        f64::sqrt(x)
    }

    /// Returns a number composed of the magnitude of `x` and the sign of `y`.
    ///
    /// Equal to `x` if the sign of `x` and `y` are the same, otherwise equal to `-x`. If `x` is a
    /// `NaN`, then a `NaN` with the sign bit of `y` is returned. Note, however, that conserving the
    /// sign bit on `NaN` across arithmetical operations is not generally guaranteed.
    #[inline(always)]
    pub fn copysign(x: f64, y: f64) -> f64 {
        f64::copysign(x, y)
    }

    /// Returns the nearest integer to `x`. If a value is half-way between two integers, round away from `0.0`.
    ///
    /// This function always returns the precise result.
    #[inline(always)]
    pub fn round(x: f64) -> f64 {
        f64::round(x)
    }

    /// Returns the largest integer less than or equal to `x`.
    ///
    /// This function always returns the precise result.
    #[inline(always)]
    pub fn floor(x: f64) -> f64 {
        f64::floor(x)
    }

    /// Returns the fractional part of `x`.
    ///
    /// This function always returns the precise result.
    #[inline(always)]
    pub fn fract(x: f64) -> f64 {
        f64::fract(x)
    }
}

#[cfg(feature = "libm")]
pub use libm_ops::*;

#[cfg(not(feature = "libm"))]
pub use std_ops::*;

#[cfg(feature = "std")]
pub use std_ops_for_no_std::*;

#[cfg(all(feature = "libm", not(feature = "std")))]
pub use libm_ops_for_no_std::*;

#[cfg(all(not(feature = "libm"), not(feature = "std")))]
compile_error!("Either the `libm` feature or the `std` feature must be enabled.");

/// This extension trait covers shortfall in determinacy from the lack of a `libm` counterpart
/// to `f64::powi`. Use this for the common small exponents.
pub trait FloatPow {
    /// Squares the f64
    fn squared(self) -> Self;
    /// Cubes the f64
    fn cubed(self) -> Self;
}

impl FloatPow for f64 {
    #[inline]
    fn squared(self) -> Self {
        self * self
    }
    #[inline]
    fn cubed(self) -> Self {
        self * self * self
    }
}
