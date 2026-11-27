# Determinism and replay

The same scenario, seed and inputs produce the same state, to the bit, on every run and platform,
natively and in WebAssembly. A recorded run replays exactly. This page shows how each source of
difference is closed, how a state is fingerprinted, and the reference run that CI checks on every
change ([#14](https://github.com/MatthewK84/MJKPhysics/issues/14)).

## Summary

- **Arithmetic:** every floating-point operation either is correctly rounded by IEEE 754 or comes
  from `libm`, which is the same Rust code on every target. Clippy rejects the standard library's
  versions.
- **Random numbers:** a hand-written xoshiro256** generator (DET-1). Each subsystem draws from its
  own stream, seeded from the run's master seed and the stream's name (DET-2), so adding a sensor
  cannot change the turbulence a run sees.
- **Fingerprints:** a 64-bit FNV-1a hash of the raw bits of everything that determines the steps
  to come (DET-5), recorded at a fixed checkpoint rate.
- **Replay:** inputs are applied between steps and logged with their tick, and a replay hands each
  one back on the same tick.
- **Proof:** the reference run below is published as a table of checkpoint hashes. Tests require
  it to match the code, and CI requires a WebAssembly build to print the same bytes as the native
  build.

## Where differences come from

| Source | Rule | Enforced by |
|---|---|---|
| Transcendental functions (`sin`, `atan2`, `exp`, `powi` and the rest) | Only through `libm`. The standard library leaves their accuracy unspecified, so results could differ between targets | Clippy `disallowed-methods` (`clippy.toml`), for `f64` and `f32` |
| Fused multiply-add | Never implicit: Rust does not contract `a * b + c`. Explicit fused multiply-adds go through `libm::fma`, since without a hardware instruction `mul_add` calls the platform's C library | The same Clippy list includes `mul_add` |
| `+`, `−`, `×`, `÷`, `sqrt` | Correctly rounded by IEEE 754 on every target, so they need no rule | — |
| Time | An integer tick count; seconds are derived, never summed ([Architecture](architecture.md#time-and-rates)) | Tests |
| Random numbers | Seeded streams only; nothing reads the clock or the operating system's entropy | Tests, review |
| Order of evaluation | One thread; subsystems run in the fixed order of `Model::update`; no hash-map iteration in the engine | Review |

The platforms agreed on the first comparison. The integrator study, which predates this page,
already matched to the bit on Linux, Windows and macOS (arm64) and in WebAssembly. Moving three
explicit fused multiply-adds from `mul_add` to `libm::fma` changed no published number, as
expected when both are correctly rounded.

## Random numbers

xoshiro256** keeps four 64-bit words $`s_0 \ldots s_3`$ and has period $`2^{256}-1`$
(Blackman and Vigna, 2021). Each draw returns

```math
x = \mathrm{rotl}(5\,s_1,\ 7)\cdot 9 \bmod 2^{64}
\qquad\text{(DET-1)}
```

and then advances the state with shifts, rotations and exclusive-ors. A stream is seeded from the
master seed $`S`$ and its name $`n`$ through `SplitMix64`, as the generator's authors recommend
(Steele, Lea and Flood, 2014). One output mixes the seed into $`m`$; a second `SplitMix64`,
seeded with the name's hash chained to $`m`$, gives four outputs $`c_1 \ldots c_4`$:

```math
m = \mathrm{SplitMix64}(S),\qquad
(c_1, c_2, c_3, c_4) = \mathrm{SplitMix64}^{4}\big(\mathrm{FNV}(n) \oplus m\big),\qquad
(s_0, s_1, s_2, s_3) = (m \oplus c_4,\ c_1,\ c_2,\ c_3)
\qquad\text{(DET-2)}
```

Each property follows from one step of the construction:

- **No two pairs share a stream.** `SplitMix64`'s first output is a bijection of its seed, so
  $`s_1`$ gives back $`\mathrm{FNV}(n) \oplus m`$, which determines $`c_4`$; then $`s_0`$ gives
  back $`m`$, so $`S`$, and with it $`\mathrm{FNV}(n)`$. Two streams start from the same state only
  if their seeds and their name hashes are both equal.
- **No word is shared.** Every word depends on both the seed and the name, so the streams of
  one run have nothing in common to correlate through.
- **Streams start differently.** The first draw depends on $`s_1`$ alone (DET-1), and $`s_1`$
  depends on both the seed and the name through a nonlinear mix. Streams in one run, and one
  stream in different runs, start with different draws.
- **Never all zero.** $`s_1`$ and $`s_2`$ come from two different `SplitMix64` states, so at most
  one of them is zero.
- **Independent of other streams.** A stream depends only on $`S`$ and $`n`$, not on how many
  other streams exist or the order they were made. The generator's jump function was not used
  because it would tie each stream to its creation order.

The first version of this page seeded a stream with $`\mathrm{SplitMix64}^{4}(S \oplus
\mathrm{FNV}(n))`$. Squeezing the seed and the name into one 64-bit value made some pairs collide:
seed 7's `gyro` stream was the same as the `pilot` stream of seed
$`7 \oplus \mathrm{FNV}(\texttt{gyro}) \oplus \mathrm{FNV}(\texttt{pilot})`$.
A split layout, with the seed in the first two words and the name in the last two, ruled that
out but gave every stream in a run the same first draw, because $`s_1`$ came from the seed
alone. A third version set $`s_0 = m`$, which every stream of a run then shared; no test found a
correlation through it, but $`s_0 = m \oplus c_4`$ removes the shared word rather than relying on
that. Tests now pin each of these properties.

Independence between streams is measured as well as argued. For every pair of the reference
run's streams, and for one stream at adjacent seeds, as a Monte Carlo sweep would use them, the
tests compare $`10^5`$ samples:

- **Correlation:** at lags from −4 to +4, the worst is 0.0074, or 2.3 standard deviations
  ($`\sigma = 1/\sqrt{10^5}`$). The tests fail above 6σ.
- **Bit balance:** for each of the 64 bits of the two streams' XOR, the worst imbalance between
  zeros and ones is 3.0σ.

Those are the largest values expected by chance from that many comparisons. Each week, PractRand
(Doty-Humphrey) tests eight streams of one run, interleaved, to 32 GB, far past what $`10^5`$
samples can see. `mjk random <seed> <name>... --bytes <n>` writes the interleaved streams.

Uniform samples take the top 53 bits of a draw:

```math
u = \big\lfloor x / 2^{11} \big\rfloor \cdot 2^{-53} \in [0, 1)
\qquad\text{(DET-3)}
```

Each of the $`2^{53}`$ values is equally likely, and the conversion is exact. Normal samples use
the Box–Muller transform (Box and Muller, 1958):

```math
z = \sqrt{-2\ln(1-u_1)}\,\cos(2\pi u_2)
\qquad\text{(DET-4)}
```

$`1 - u_1`$ lies in $`(0, 1]`$, so the logarithm is finite. Each sample draws two uniforms and
keeps one normal. Keeping the second normal would save draws but would add a cached value to the
generator's state.

## State fingerprints

A `StateHasher` folds bytes $`b_i`$ into a 64-bit FNV-1a hash, little-endian on every platform:

```math
h_0 = \mathtt{cbf29ce484222325}_{16},\qquad
h_{i+1} = (h_i \oplus b_i)\cdot \mathtt{100000001b3}_{16} \bmod 2^{64}
\qquad\text{(DET-5)}
```

Each `f64` goes in as its raw bits, so `0.0` and `-0.0` hash differently. Each step is a
bijection of $`h`$, so changing any single byte always changes the hash. FNV-1a is a fingerprint
for comparing honest runs, not a cryptographic hash.

`World::state_hash` covers everything that determines the steps to come:
- the clock (tick and rate);
- the current state;
- the model's own state: held commands, sensor readings and the state of every random stream.

A model provides the last of these by implementing `StateHash` with every field that can affect
the future, in a fixed order. The previous state and the real-time remainder affect only what is
drawn, so they are left out.

## Checkpoints and replay

`Checkpoints` records the hash at a fixed rate, such as once a second, from tick 0. An input
changes a model between steps: `World::apply` logs it with the current tick, then applies it, and
it holds from the next step on. `World::replay_due`, called before each step, applies the logged
inputs for that tick in their recorded order:

```rust,ignore
let mut world = World::new(model, Rkmk4, y0, 1000)?;
let mut replay = Replay::new(&log);
for _ in 0..ticks {
    world.replay_due(&mut replay);
    world.checkpoint(&mut checkpoints)?;
    world.step();
}
```

Both records are sized when they are made and never grow, so recording allocates nothing inside a
step. The [allocation test](engineering-standards.md#proving-a-step-allocates-nothing) records
1000 checkpoints and 1000 inputs, then replays them. A full record, or a checkpoint that is not
later than the last one, is an error, never a silent drop. An input that cannot be logged is not
applied, so the log always describes the run.

## The reference run

A tumbling 2 kg body is flown by random pilot commands through a rate controller that reads a noisy
gyro, with a noisy accelerometer alongside. It is integrated with RKMK4 at 1 kHz for 10 s, with a
checkpoint every second.

| Parameter | Value |
|---|---|
| Master seed | 14 |
| Streams | `gyro`, `accelerometer`, `pilot` |
| Gyro | 500 Hz, white noise σ = 0.01 rad/s |
| Accelerometer | 250 Hz, white noise σ = 0.05 m/s² |
| Rate controller | 250 Hz, $`\mathbf{M} = k\,\mathbf{J}\,(\boldsymbol{\omega}_{cmd} - \tilde{\boldsymbol{\omega}})`$, $`k`$ = 8 s⁻¹ |
| Pilot | A command every 0.5 s: body rates with σ = 0.5 rad/s, thrust within σ = 5 % of hover |
| Initial state | At rest 10 m up, ω = (0.4, −0.3, 1.5) rad/s |

The run's state hash at each checkpoint, beside the hash from a replay of its 20 recorded
commands:

{{#include validation/determinism.md}}

*Generated by `mjk validate determinism`. A test fails if this table no longer matches the code,
and CI fails unless the native build and a WebAssembly build print it byte for byte.*

The two hash columns are equal at every checkpoint, so replaying the 20 recorded commands
reproduces the run, final state included. Running the scenario twice gives the same table. Tests
also show that the hashes respond to every change:
- another seed changes every checkpoint;
- raising one command's thrust by one part in $`10^9`$ changes every checkpoint from that command's
  tick on, and none before it.

**A fingerprint is exact, so it also shows what rounding absorbs.** Changing a 0.01 N·m moment
command by one ulp did not change the final hash. Held for 0.25 s on a 0.02 kg·m² axis, the
change moves ω by about $`2\times10^{-17}`$ rad/s. That is below the resolution of an ω near
1 rad/s, so once the next command replaced it, the two runs were in the same state to the bit.

## Validation

| Check | Criterion | Test |
|---|---|---|
| `SplitMix64`, xoshiro256** and FNV-1a against the published reference algorithms | every value equal | `splitmix64_matches_the_reference`, `xoshiro256_star_star_matches_the_reference`, `the_hasher_is_fnv_1a_64` |
| A new stream leaves another's sequence unchanged | 100 draws identical, interleaved or alone | `adding_a_stream_does_not_change_another` |
| No two seed-and-name pairs share a stream; the state is built as in DET-2 | the pair that collided under XOR seeding now differs; state equal word for word | `no_two_seed_and_name_pairs_share_a_stream`, `a_stream_state_is_the_name_chained_to_the_mixed_seed` |
| No state word is shared by the streams of a run | all four words differ, pair by pair | `no_word_is_shared_by_the_streams_of_a_run` |
| Streams of one run, and one stream at adjacent seeds, are uncorrelated | correlation at lags −4 to +4 below 0.019 (6σ); every bit of their XOR balanced within 6σ, over $`10^5`$ samples | `streams_in_a_run_are_uncorrelated`, `adjacent_seeds_are_uncorrelated`; `the_independence_checks_detect_a_dependent_stream` shows both checks fail on a dependent pair |
| Eight interleaved streams of one run | no PractRand failure to 32 GB | weekly `deep-analysis.yml`, job *Random streams (PractRand)* |
| Streams start with different draws | distinct first draws for three streams in one run, and for one stream over three seeds | `streams_start_with_different_draws` |
| The reference run's stream names | distinct FNV-1a hashes | `every_stream_name_hashes_differently` |
| Uniform samples: endpoints and moments | exact at 0 and $`1-2^{-53}`$; mean and variance within 0.005 and 0.002 | `a_uniform_sample_is_the_top_53_bits_scaled_into_the_unit_interval`, `uniform_samples_have_the_right_mean_and_spread` |
| Normal samples: values and moments | within $`10^{-14}`$ of a reference; variance within 0.03 of 1; 68.27 % within one σ | `gaussian_samples_match_the_box_muller_reference`, `gaussian_samples_have_unit_variance_and_normal_tails` |
| Two runs of the reference scenario | identical at every checkpoint | `two_runs_agree_at_every_checkpoint` |
| Replaying the recorded inputs | identical checkpoints and final hash | `replaying_the_inputs_reproduces_the_final_hash` |
| An altered input | every hash from its tick on differs, none before | `an_altered_input_changes_every_hash_from_its_tick_on` |
| Native and WebAssembly (`wasm32-wasip1` under Node) | both studies' output identical to each other and to the book | CI job *Native and WebAssembly agree* |
| Windows and macOS (arm64) | the published tables reproduce | weekly `cross-platform.yml` |
| Recording and replay | no allocation | `allocations` (harness-free test) |

## Limitations

- **Logs live in memory.** Writing them to a file and `mjk replay` arrive with the command-line
  tool ([#17](https://github.com/MatthewK84/MJKPhysics/issues/17)).
- **Only logged inputs replay.** A change made through `World::model_mut` bypasses the log.
- **NaN payloads.** WebAssembly may give a NaN a different payload or sign than native code does.
  A NaN in the state already means the run has failed (a degenerate attitude makes every rate NaN,
  see [Integrators](models/integrators.md#implementation-notes)), so only failed runs can hash
  differently.
- **Targets.** Supported targets compute in IEEE 754 double precision: x86-64, arm64 and
  WebAssembly. Targets that compute with the x87's extended precision, such as `i586`, are not
  supported.

## References

- D. Blackman, S. Vigna, "Scrambled linear pseudorandom number generators," *ACM Transactions on
  Mathematical Software*, 2021. Reference code: `xoshiro256starstar.c`, `splitmix64.c`.
- G. L. Steele Jr., D. Lea, C. H. Flood, "Fast splittable pseudorandom number generators,"
  *OOPSLA*, 2014.
- G. E. P. Box, M. E. Muller, "A note on the generation of random normal deviates," *Annals of
  Mathematical Statistics* 29(2), 1958.
- G. Fowler, L. C. Noll, K.-P. Vo, D. Eastlake, T. Hansen, "The FNV non-cryptographic hash
  algorithm," IETF Internet-Draft `draft-eastlake-fnv`.
- IEEE 754-2019, *Standard for Floating-Point Arithmetic*: correct rounding of the basic
  operations, square root and fused multiply-add.
- C. Doty-Humphrey, *PractRand*, version 0.96, a test suite for random number generators
  (pracrand.sourceforge.net).
