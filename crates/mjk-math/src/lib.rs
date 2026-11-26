//! Frame-tagged linear algebra and quaternions.
//!
//! Lowest math layer of the engine, hand-written with no third-party math crates (ADR 0001).
//! Every vector, matrix and rotation carries its reference frame in its type, so mixing frames
//! is a compile error rather than a wrong answer:
//!
//! - [`Vec3<F>`] — a vector resolved in frame `F`.
//! - [`Mat3<F>`] — a linear operator on `Vec3<F>`, such as an inertia tensor.
//! - [`Rotation<To, From>`] — maps `Vec3<From>` to `Vec3<To>`; `Rotation<World, Body>` is the
//!   attitude $`\mathbf{q}_{WB}`$.
//! - [`Quat`] and [`UnitQuat`] — Hamilton quaternions, scalar first, without frames.
//!
//! Frame markers are [`World`], [`Body`], [`Stability`] and [`Wind`]. Conventions follow the
//! book's conventions page; code comments cite its equations, such as Eq. (CONV-8).
//!
//! Only `+ − × ÷` and `sqrt` are used, which IEEE 754 rounds exactly, so results are
//! bit-identical on every platform. Transcendental functions arrive with the SO(3) maps (#9)
//! and go through `libm`.

mod eigen;
mod frame;
mod mat3;
mod quat;
mod rotation;
mod vec3;

pub use eigen::{JACOBI_TOLERANCE, MAX_JACOBI_SWEEPS, SymmetricEigen};
pub use frame::{Body, Frame, Stability, Wind, World};
pub use mat3::Mat3;
pub use quat::{Quat, UnitQuat};
pub use rotation::Rotation;
pub use vec3::Vec3;
