# Model page template

Every physical model gets a page in `docs/src/models/`, built from this template. Reserve an
equation prefix in the [models registry](../models/README.md) in the same pull request.

````markdown
# Model name

- **Issue:** #NN
- **Code:** `crate::module` (link to the source)
- **Equation prefix:** `ABC`

## Summary

What the model computes, what it is for, and which fidelity tier it belongs to.

## Assumptions

Numbered, specific and checkable, for example: "Inflow is uniform across the disk."

## Equations

Symbols follow the conventions page. Each display equation ends with its ID:

```math
T = C_{T,\text{prop}}\,\rho\,n^2 D^4 \qquad\text{(ABC-1)}
```

Inline math is written $`\rho`$. Derivations go here, or are cited when they are standard.

## Validity range

Where the model holds, with numbers, and what the engine does outside that range.

## Parameters

| Symbol | Meaning | Unit | Typical value | Source |
|---|---|---|---|---|

## Implementation notes

Numerical method, iteration bounds, guards, and how the code maps to the equations: cite
`// Eq. (ABC-1)` in the code.

## Validation

The tests that check this model, their tolerances and results, and known limitations.

## References

Full citations.
````
