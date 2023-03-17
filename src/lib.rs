#[cfg(feature = "f64")]
pub mod double;

#[cfg(feature = "f32")]
pub mod single;

#[cfg(all(feature = "f64", not(feature = "f32")))]
pub use double::*;

#[cfg(all(feature = "f32", not(feature = "f64")))]
pub use single::*;
