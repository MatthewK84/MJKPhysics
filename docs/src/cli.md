# Command-line tool

`mjk` runs the engine without a window. It writes the validation tables this book includes, flies
the reference scenario and writes its log, and replays a log to check it to the bit
([#17](https://github.com/MatthewK84/MJKPhysics/issues/17)). Engine crates do no I/O, so `mjk`
reads and writes every file and turns engine errors into messages and exit codes.

## Commands

| Command | What it does |
|---|---|
| `mjk validate [--out <dir>]` | Runs every validation study and writes each table into `<dir>`, by default `docs/src/validation`, the book's copy. Lists each table with `pass` or `FAIL`. |
| `mjk validate <study>` | Prints one study's table: `integrators`, `determinism`, `atmosphere` or `analytic`. |
| `mjk validate integrators --timing` | Prints the wall-clock time per step of each integration method. These numbers depend on the machine. |
| `mjk run reference [--seed <n>] [--log <file.json>] [--csv <file.csv>]` | Flies the [reference scenario](determinism.md#the-reference-run) for 10 s and prints a summary. Writes the run log as JSON and the state log as CSV. |
| `mjk replay <file.json>` | Flies a recorded run again with its recorded inputs and checks every sample and fingerprint. |
| `mjk random <seed> <name>... --bytes <n>` | Writes named random streams for statistical test suites ([Determinism](determinism.md#random-numbers)). |

A file named `-` is standard input or standard output. When a log goes to standard output, the
summary moves to standard error. Scenario files, and with them `mjk run <scenario.toml>`, arrive
with [#28](https://github.com/MatthewK84/MJKPhysics/issues/28).

Record the reference run, then replay it:

```bash
cargo run -p mjk-cli --release -- run reference --log run.json --csv run.csv
```

```bash
cargo run -p mjk-cli --release -- replay run.json
```

Regenerate the book's validation tables from the repository root:

```bash
cargo run -p mjk-cli --release -- validate
```

## Exit codes

| Code | Meaning |
|---:|---|
| 0 | Done: every check passed, or the replay matched its record. |
| 1 | Done, but a validation check failed or the replay diverged. The table or report is still written. |
| 2 | The command line is wrong. The usage follows the message. |
| 3 | An input file is malformed, unsupported or physically implausible: an `EngineError::Config`. The message names the file and the field, for example `run.json: inputs[2].tick`. |
| 4 | The simulation failed: a numerical or convergence error, or a built-in scenario that could not run. |
| 5 | A file could not be read or written. |

Every error is one line on standard error, starting `error:`.

## Run logs

A run log is one JSON document holding what it takes to fly the run again and check it:

| Field | Contents |
|---|---|
| `format_version` | `1` |
| `scenario` | `"reference"` |
| `seed` | The master seed of every random stream |
| `physics_hz`, `ticks` | The physics rate and the run's length in steps: 1000 Hz and 10 000 |
| `sample_hz` | Rows of the state log per second: 100 |
| `inputs` | Each pilot command, in order: `tick`, `rate_rad_s` and `thrust_n` |
| `checkpoints` | The state fingerprint once a second: `tick` and `hash` |
| `final_hash` | The fingerprint after the last step |
| `channels` | The state log, one entry per channel: `name`, `unit` and `values` |

The state log has 14 channels and 1001 rows, at tick 0 and every 10 ticks to the end:

| Channels | Unit | Quantity |
|---|---|---|
| `time` | s | Simulation time |
| `north`, `east`, `down` | m | Position, world axes (NED) |
| `velocity_north`, `velocity_east`, `velocity_down` | m/s | Velocity, world axes |
| `attitude_w`, `attitude_x`, `attitude_y`, `attitude_z` | 1 | Attitude quaternion, body to world |
| `roll_rate`, `pitch_rate`, `yaw_rate` | rad/s | Body angular velocity |

**Numbers read back exactly.**
- **How numbers are written:** each one is the shortest decimal that reads back to the same
  double.
- **How they are read:** `mjk` parses them exactly. `serde_json`'s default float parser is
  not exact, so a test checks 10⁵ random doubles; with the default parser, the first one that
  comes back wrong is `-2.1513850520192137e121`.
- **Non-finite samples:** JSON has no NaN or infinity, so these are written as the strings
  `"NaN"`, `"inf"` and `"-inf"`.
- **Fingerprints:** these are 16 hexadecimal digits, because many JSON readers lose integers
  above 2⁵³.

The CSV export has a header of `name (unit)` fields and one line per row. It uses the same
shortest decimals, with an exponent for very small and very large values, and `NaN`, `inf` and
`-inf`.

## How a replay decides

1. **Refuse what it cannot fly.** Each of these exits with code 3 and names the field:
   - another format version, scenario, rate or length;
   - other channels;
   - inputs out of order;
   - a fingerprint that is not 16 hexadecimal digits;
   - a record with fewer checkpoints or samples than the run produces. That is a truncated
     file, not a divergence.
2. **Fly it again** from the recorded seed, applying each recorded input on its tick.
3. **Compare everything:** every sample bit for bit (NaN equals NaN), every checkpoint, and the
   final fingerprint.
4. **Report the earliest difference.** If a sample and a fingerprint differ at the same tick, the
   sample is reported, because it names the quantity.

Changing one pilot command's thrust by one part in 10⁹ at tick 500 is found at tick 510, the
first sample after the change:

```text
replay diverged at tick 510: north was 0.06303508229555076 and is 0.06303508229555473 on replay
```

## Across platforms

CI's *Native and WebAssembly agree* job records the reference run natively and as a
WebAssembly module (`wasm32-wasip1` under Node), and requires the two logs to be identical byte
for byte. It then replays each log with the other build. A run recorded on one platform
reproduces on the other to the bit, as [Determinism and replay](determinism.md) promises.

## The log writer

The state log is a `ColumnLog` in `mjk_sim::log`.
- **Storage:** it has a fixed list of channels, each with its own column, sized for the whole run
  when it is made.
- **Logging a row allocates nothing.** A test logs 10⁵ rows under a counting allocator and
  finds no allocation.
- **A full log is an error, never a silent drop,** as with checkpoints and recorded inputs.
- **Writing happens off the step path.** The log is written after the run, so writing never
  slows a step.

## Limitations

- **Logs live in memory until the run ends.** The reference run's log is 261 KB. Streaming
  long runs to disk, in MCAP with typed channels, is
  [#79](https://github.com/MatthewK84/MJKPhysics/issues/79).
- **One scenario.** `run` and `replay` know only the built-in reference scenario until scenario
  files arrive with [#28](https://github.com/MatthewK84/MJKPhysics/issues/28), which also widens
  the log's inputs beyond pilot commands.
- **A log does not record the build that wrote it.** A build with different physics replays it
  and reports a divergence, as it should, but cannot say which build recorded the log.
