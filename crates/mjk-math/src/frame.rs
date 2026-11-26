//! Reference-frame marker types.
//!
//! A frame marker is a zero-sized type that tags vectors, matrices and rotations with the frame
//! their components are resolved in, so the compiler rejects mixing frames. The frames are
//! defined on the conventions page of the book.

use std::fmt::Debug;

/// A reference frame. Implemented by zero-sized marker types.
pub trait Frame: Copy + Clone + Debug + PartialEq + Eq + 'static {
    /// Short name used in debug output, such as `"World"`.
    const NAME: &'static str;
}

/// World frame: local-level North-East-Down, fixed to the ground.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct World;

/// Body frame: Forward-Right-Down, fixed to the airframe at the center of mass.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Body;

/// Stability axes: body axes rotated by the angle of attack (CONV-3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stability;

/// Wind (air-relative) axes: x along the relative wind (CONV-3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Wind;

impl Frame for World {
    const NAME: &'static str = "World";
}

impl Frame for Body {
    const NAME: &'static str = "Body";
}

impl Frame for Stability {
    const NAME: &'static str = "Stability";
}

impl Frame for Wind {
    const NAME: &'static str = "Wind";
}
