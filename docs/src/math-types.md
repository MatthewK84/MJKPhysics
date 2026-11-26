# Math types

`mjk-math` is the hand-written linear algebra underneath the engine
([ADR 0001](adr/0001-hand-written-math.md)). Its defining feature is that every vector, matrix
and rotation carries its reference frame in its type, so a frame mistake fails to compile instead
of producing a plausible wrong answer.

## The types

| Type | Meaning | Notation |
|---|---|---|
| `Vec3<F>` | A vector resolved in frame `F` | $`\mathbf{v}^F`$ |
| `Mat3<F>` | A linear operator on `Vec3<F>`, such as an inertia tensor in body axes | $`\mathbf{J}^B`$ |
| `Rotation<To, From>` | Maps `Vec3<From>` to `Vec3<To>` | $`\mathbf{q}_{\text{To},\text{From}}`$, $`\mathbf{R}_{\text{To},\text{From}}`$ |
| `Quat` | A general Hamilton quaternion, scalar first, for algebra such as $`\dot{\mathbf{q}}`$ | $`\mathbf{q}`$ |
| `UnitQuat` | A unit quaternion: a rotation with no frame attached | $`\lVert\mathbf{q}\rVert = 1`$ |

The frame markers are `World`, `Body`, `Stability` and `Wind`, as defined in the
[conventions](conventions.md#2-reference-frames). Other crates can define more frames, such as a
rotor hub frame, by implementing the `Frame` trait on a marker type.

## Types that read like the notation

`Rotation<To, From>` puts the target frame first, matching the subscripts in the conventions:
`q_wb: Rotation<World, Body>` is $`\mathbf{q}_{WB}`$, and `q_wb.rotate(v_b)` is
$`\mathbf{x}^W = \mathbf{R}_{WB}\,\mathbf{x}^B`$. Composition type-checks exactly when the inner
subscripts cancel:

```rust,ignore
let q_ws: Rotation<World, Stability> = q_wb * q_bs;   // W←B ∘ B←S = W←S
```

`Mat3<F>` only acts on vectors in its own frame. Moving a tensor to another frame goes through a
rotation, `q_wb.transform(j_b)`, which computes $`\mathbf{R}\,\mathbf{J}\,\mathbf{R}^\top`$.

## What the compiler rejects

Each of these is a `compile_fail` documentation test, so CI proves it stays a compile error:

| Mistake | Compiler error |
|---|---|
| Adding a body-frame vector to a world-frame vector | `expected Vec3<Body>, found Vec3<World>` |
| Rotating a vector from the wrong frame with `q_wb.rotate(v_w)` | `expected Vec3<Body>, found Vec3<World>` |
| Composing rotations whose inner frames differ, `q_wb * q_ws` | `expected Rotation<Body, _>, found Rotation<World, Stability>` |
| Applying a body-frame inertia tensor to a world-frame rate | `no implementation for Mat3<Body> * Vec3<World>` |

Construction is where meaning is asserted: `Vec3::new` and `Rotation::from_unit_quat` take the
frame from the type the caller writes. After that, the compiler keeps the frames straight.

## Numerics

- **Bit-identical everywhere.** Arithmetic and square roots are rounded exactly by IEEE 754, and
  every transcendental function (`sin`, `cos`, `atan2`) comes from `libm`, so results match on
  every platform, including WebAssembly. The maps built on them are on the
  [SO(3) maps](so3.md) page.
- **Rotating a vector** uses $`\mathbf{v}' = \mathbf{v} + 2q_0(\mathbf{q}_v\times\mathbf{v}) + 2\,\mathbf{q}_v\times(\mathbf{q}_v\times\mathbf{v})`$,
  which equals $`\mathbf{q}\otimes\mathbf{v}\otimes\mathbf{q}^{\ast}`$ for a unit quaternion and is
  tested against the matrix form (CONV-8).
- **Inverse.** `Mat3::inverse` uses the adjugate (cross products of columns over the
  determinant) and returns `None` for singular or non-finite matrices.
- **Symmetric eigen-decomposition.** `Mat3::symmetric_eigen` uses the cyclic Jacobi method
  (Golub & Van Loan, §8.5). It stops when the off-diagonal norm falls below $`10^{-15}`$ times the
  largest entry. Convergence is quadratic, so 3×3 matrices need a few sweeps; after 32 sweeps it
  returns a convergence error rather than looping. Eigenvalues come back in ascending order, with
  eigenvectors forming a right-handed orthonormal basis, ready to serve as principal axes.

## How it is tested

- **Property tests** (proptest, 512 random cases per property per run):
  - rotations preserve norm, and $`\mathbf{q}\otimes\mathbf{q}^{-1} = 1`$;
  - quaternion composition equals the matrix product, and quaternion rotation equals its matrix;
  - typed composition applies the right operand first, and inverses round-trip;
  - tensor transforms act consistently;
  - the cross product is antisymmetric, bit for bit, and orthogonal to its operands;
  - skew and outer products, transpose, determinant and inverse behave as they should;
  - for symmetric matrices: eigenpairs satisfy $`\mathbf{A}\mathbf{v} = \lambda\mathbf{v}`$, are
    orthonormal and right-handed, and rebuild the matrix.
- **Mutation checks.** Flipping a sign in the Hamilton product, or in either Jacobi update, makes
  the properties fail. This confirms they actually constrain the implementation.
- **`compile_fail` tests** for every frame mistake in the table above.

## References

- G. H. Golub, C. F. Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins University Press,
  2013, §8.5.
- J. Solà, "Quaternion kinematics for the error-state Kalman filter," arXiv:1711.02508, 2017.
