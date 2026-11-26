//! Frame-tagged linear algebra, quaternions and SO(3) maps.
//!
//! Lowest layer of the engine: it depends on nothing else in the workspace and uses no
//! third-party math crates. Vectors will carry their reference frame in the type
//! (`Vec3<Body>` versus `Vec3<World>`) so frame mixing is a compile error.
//!
//! Planned work: frame-tagged types ([#8](https://github.com/MatthewK84/MJKPhysics/issues/8)) and SO(3) exponential and
//! logarithm maps ([#9](https://github.com/MatthewK84/MJKPhysics/issues/9)). Conventions are fixed in [#5](https://github.com/MatthewK84/MJKPhysics/issues/5).
