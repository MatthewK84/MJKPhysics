# ADR 0003: Sequential impulses for contact

- **Status:** Accepted
- **Date:** 2026-11-26
- **Issues:** [#55](https://github.com/MatthewK84/MJKPhysics/issues/55), [#56](https://github.com/MatthewK84/MJKPhysics/issues/56), [#57](https://github.com/MatthewK84/MJKPhysics/issues/57), [#59](https://github.com/MatthewK84/MJKPhysics/issues/59), [#61](https://github.com/MatthewK84/MJKPhysics/issues/61)

## Context

Aircraft touch the world when landing, taking off from slopes, launching, crashing, and towing
slung payloads. The solver must:

- keep a landed vehicle at rest without jitter;
- handle Coulomb friction;
- support joints such as rope constraints for slung loads;
- be deterministic and allocation-free per step;
- couple cleanly with velocity-level rigid-body dynamics.

The scenes are small: a few bodies and tens of contacts.

## Decision

Contacts and joints are velocity constraints solved by **sequential impulses** (projected
Gauss–Seidel):

- warm starting from persistent contact manifolds;
- Coulomb friction with two tangent directions, bounded by the normal impulse;
- restitution above a velocity threshold;
- position error corrected with **soft constraints**, in the style of Box2D's soft step, rather
  than raw Baumgarte stabilization;
- sleeping.

Collision detection uses GJK and EPA on convex shapes, with analytic fast paths, plus a dynamic
AABB tree for the broadphase.

## Consequences

- Cheap, robust, and well understood. Each impulse has a physical meaning that can be logged and
  plotted.
- Iterative: the residual depends on the iteration count, which is logged every step.
- Known limitations are documented and tested rather than hidden: slow convergence for large mass
  ratios, and Newton's cradle (simultaneous impacts are resolved sequentially).

## Alternatives considered

- **Penalty springs.** Simple, but stiff springs need tiny steps or they jitter and sink, and
  soft springs let bodies interpenetrate.
- **Direct LCP solvers (Lemke, Dantzig).** Exact for the linear complementarity problem, but more
  complex and costly, and their pivoting is harder to make deterministic, for little benefit at
  this scale.
- **Position-based dynamics (XPBD).** Very stable, but its corrections are less directly
  interpretable as physical impulses, and coupling it with force-based aerodynamics is less
  natural.
- **Optimization-based soft contact (MuJoCo-style convex formulation).** Elegant and accurate,
  but substantially more machinery than a few-body flight simulator needs.
