//! The motor and ESC study of [#19](https://github.com/MatthewK84/MJKPhysics/issues/19): the
//! models through the simulation loop, `World` with RKMK4 at 1 kHz and the ESC updated every
//! tick, against their closed forms, Eqs. (MOT-5) to (MOT-13).
//!
//! The test motor is an illustrative 2212-class outrunner, 920 rpm/V, 0.117 Ω and 0.4 A no-load
//! current, turning 8.5 × 10⁻⁵ kg·m² of rotor and 10-inch propeller from an 11.1 V supply. Each
//! tolerance comes from an error estimate computed before the run, as in the analytic suite.

use core::f64::consts::PI;
use std::fmt;

use mjk_aero::propulsion::{
    Braking, ConstantLoad, DcMotor, Drive, DriveInput, Esc, EscError, EscState, LagMotor,
    LoadTorque, MotorError, MotorModel, MotorSpeed, MotorSystem, PropellerLoad,
};
use mjk_dynamics::{Clock, Model, RateError, Rkmk4, World};

use super::check::{Check, tolerance_from};

pub use super::check::markdown;

/// The physics rate.
pub const PHYSICS_HZ: u32 = 1000;

/// The physics step, s.
const STEP_S: f64 = 1.0 / 1000.0;

/// The supply voltage, V.
const SUPPLY_V: f64 = 11.1;

/// The propeller's torque coefficient, N·m·s².
const PROPELLER_K: f64 = 1.3e-7;

const EPSILON: f64 = f64::EPSILON;

/// 1 − e⁻¹: the fraction of a first-order step complete after one time constant.
const ONE_TIME_CONSTANT: f64 = 0.632_120_558_828_557_7;

/// Why the study could not run. None of these happen with its fixed configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotorStudyError {
    /// A motor's parameters are not usable.
    Motor(MotorError),
    /// An ESC's parameters are not usable.
    Esc(EscError),
    /// The physics rate is not usable.
    Rate(RateError),
}

impl fmt::Display for MotorStudyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Motor(error) => write!(f, "the test motor is not usable: {error}"),
            Self::Esc(error) => write!(f, "the test ESC is not usable: {error}"),
            Self::Rate(error) => write!(f, "the physics rate is not usable: {error}"),
        }
    }
}

impl From<MotorError> for MotorStudyError {
    fn from(error: MotorError) -> Self {
        Self::Motor(error)
    }
}

impl From<EscError> for MotorStudyError {
    fn from(error: EscError) -> Self {
        Self::Esc(error)
    }
}

impl From<RateError> for MotorStudyError {
    fn from(error: RateError) -> Self {
        Self::Rate(error)
    }
}

/// The test motor.
///
/// # Errors
///
/// None with these parameters; the type is the constructor's.
pub fn test_motor() -> Result<DcMotor, MotorError> {
    DcMotor::new(920.0 * 2.0 * PI / 60.0, 0.117, 0.4, 8.5e-5)
}

/// An ESC with no latency or slew limit.
fn ideal_esc(braking: Braking, current_limit_a: Option<f64>) -> Result<Esc, EscError> {
    let drive = Drive {
        braking,
        current_limit_a,
    };
    Esc::new(0.0, None, drive, PHYSICS_HZ)
}

/// A motor on a test bench: its ESC runs every tick with the bench's command.
struct Bench<L: LoadTorque> {
    system: MotorSystem<L>,
    esc: EscState,
    command: f64,
}

impl<L: LoadTorque> Model for Bench<L> {
    type System = MotorSystem<L>;

    fn system(&self) -> &MotorSystem<L> {
        &self.system
    }

    fn update(&mut self, _: &Clock, _: &MotorSpeed) {
        self.system.input.duty = self.esc.update(self.command);
    }
}

type BenchWorld<L> = World<Bench<L>, Rkmk4>;

/// How a run starts: the speed, the ESC's duty cycle, and the command from tick 0.
#[derive(Clone, Copy)]
struct Start {
    speed_rad_s: f64,
    duty: f64,
    command: f64,
}

fn bench<L: LoadTorque>(
    motor: MotorModel,
    load: L,
    esc: &Esc,
    start: Start,
) -> Result<BenchWorld<L>, MotorStudyError> {
    let state = esc.start(start.duty);
    let input = DriveInput {
        duty: state.duty(),
        supply_v: SUPPLY_V,
        drive: esc.drive(),
    };
    let model = Bench {
        system: MotorSystem { motor, load, input },
        esc: state,
        command: start.command,
    };
    let y0 = MotorSpeed {
        speed_rad_s: start.speed_rad_s,
    };
    Ok(World::new(model, Rkmk4, y0, PHYSICS_HZ)?)
}

/// The speed after `steps` steps.
fn speed_after<L: LoadTorque>(world: &mut BenchWorld<L>, steps: u32) -> f64 {
    for _ in 0..steps {
        world.step();
    }
    world.current().speed_rad_s
}

fn count(steps: u32) -> f64 {
    f64::from(steps)
}

/// Steady speed under three constant loads, from rest at full duty, Eq. (MOT-5).
fn steady_speeds(motor: DcMotor) -> Result<Vec<Check>, MotorStudyError> {
    let esc = ideal_esc(Braking::Active, None)?;
    let steps = 3000;
    let tau = motor.time_constant_s(0.0);
    let start = Start {
        speed_rad_s: 0.0,
        duty: 1.0,
        command: 1.0,
    };
    [0.0, 0.02, 0.05]
        .into_iter()
        .map(|load| {
            let closed = motor.steady_speed(SUPPLY_V, load);
            let mut world = bench(
                MotorModel::Dc(motor),
                ConstantLoad { torque_n_m: load },
                &esc,
                start,
            )?;
            let simulated = speed_after(&mut world, steps);
            let transient = closed * libm::exp(-count(steps) * STEP_S / tau);
            let rounding = count(steps) * EPSILON * closed;
            let estimate = transient + rounding;
            let why = format!(
                "The transient left after T = 3 s, ω·e^(−T/τ) = {transient:.1e} with τ = J·R·K_v² \
                 = {tau:.4} s, plus rounding N·ε·ω = {rounding:.1e} over N = {steps} steps: \
                 {estimate:.1e} rad/s."
            );
            let name = format!("Steady speed, constant load {load} N·m");
            Ok(Check::new(&name, closed, simulated, "rad/s")
                .with_tolerance(tolerance_from(estimate), why))
        })
        .collect()
}

/// Steady speed under a propeller load at 80 % duty, Eq. (MOT-7).
fn propeller_steady_speed(motor: DcMotor) -> Result<Check, MotorStudyError> {
    let esc = ideal_esc(Braking::Active, None)?;
    let load = PropellerLoad {
        coefficient_n_m_s2: PROPELLER_K,
    };
    let closed = motor.steady_speed_quadratic(0.8 * SUPPLY_V, PROPELLER_K);
    let tau = motor.time_constant_s(load.slope(closed));
    let steps = 3000;
    let start = Start {
        speed_rad_s: 0.0,
        duty: 0.8,
        command: 0.8,
    };
    let mut world = bench(MotorModel::Dc(motor), load, &esc, start)?;
    let simulated = speed_after(&mut world, steps);
    let transient = closed * libm::exp(-count(steps) * STEP_S / tau);
    let rounding = count(steps) * EPSILON * closed;
    let estimate = transient + rounding;
    let why = format!(
        "The transient left after T = 3 s with the linearized τ = {tau:.4} s, Eq. (MOT-6), \
         ω·e^(−T/τ) = {transient:.1e}, plus rounding N·ε·ω = {rounding:.1e}: {estimate:.1e} \
         rad/s. The transient decays faster than e^(−T/τ) from below."
    );
    Ok(Check::new(
        "Steady speed, propeller load, 80 % duty",
        closed,
        simulated,
        "rad/s",
    )
    .with_tolerance(tolerance_from(estimate), why))
}

/// The time at which the speed first reaches `level`, interpolated linearly between steps.
fn crossing_time<L: LoadTorque>(world: &mut BenchWorld<L>, level: f64, max_steps: u32) -> f64 {
    let mut before = (world.clock().time_s(), world.current().speed_rad_s);
    for _ in 0..max_steps {
        world.step();
        let after = (world.clock().time_s(), world.current().speed_rad_s);
        if after.1 >= level {
            return before.0 + (level - before.1) / (after.1 - before.1) * (after.0 - before.0);
        }
        before = after;
    }
    f64::NAN
}

/// The small-signal time constant: a 1 % step of duty at the propeller operating point, Eq. (MOT-6).
fn small_signal_time_constant(motor: DcMotor) -> Result<Check, MotorStudyError> {
    let esc = ideal_esc(Braking::Active, None)?;
    let load = PropellerLoad {
        coefficient_n_m_s2: PROPELLER_K,
    };
    let (low, high) = (0.8, 0.808);
    let initial = motor.steady_speed_quadratic(low * SUPPLY_V, PROPELLER_K);
    let last = motor.steady_speed_quadratic(high * SUPPLY_V, PROPELLER_K);
    let tau = motor.time_constant_s(load.slope(last));
    let start = Start {
        speed_rad_s: initial,
        duty: low,
        command: high,
    };
    let mut world = bench(MotorModel::Dc(motor), load, &esc, start)?;
    let level = initial + ONE_TIME_CONSTANT * (last - initial);
    let measured = crossing_time(&mut world, level, 1000);
    let nonlinearity = PROPELLER_K * (last - initial) * tau / motor.inertia_kg_m2();
    let interpolation = STEP_S * STEP_S / (8.0 * tau * tau);
    let estimate = nonlinearity + interpolation;
    let why = format!(
        "Relative: the quadratic load's second-order term, k·Δω·τ/J = {nonlinearity:.1e} for a \
         step of Δω = {:.2} rad/s, plus linear interpolation of the crossing, h²/(8τ²) = \
         {interpolation:.1e}: {estimate:.1e}.",
        last - initial
    );
    Ok(Check::new(
        "Small-signal time constant, 1 % step at 80 % duty",
        tau,
        measured,
        "s",
    )
    .relative()
    .with_tolerance(tolerance_from(estimate), why))
}

/// The lag tier's time constant: a step from rest to half speed, Eq. (MOT-8).
fn lag_time_constant() -> Result<Check, MotorStudyError> {
    let tau = 0.03;
    let lag = LagMotor::new(tau, 2500.0)?;
    let esc = ideal_esc(Braking::Active, None)?;
    let start = Start {
        speed_rad_s: 0.0,
        duty: 0.0,
        command: 0.5,
    };
    let mut world = bench(
        MotorModel::Lag(lag),
        ConstantLoad { torque_n_m: 0.0 },
        &esc,
        start,
    )?;
    let measured = crossing_time(&mut world, ONE_TIME_CONSTANT * 1250.0, 1000);
    let z = STEP_S / tau;
    let estimate = z * z / 8.0;
    let truncation = z * z * z * z / 120.0;
    let why = format!(
        "Relative: linear interpolation of the crossing, h²/(8τ²) = {estimate:.1e} with τ = \
         30 ms. RK4's truncation of e^(−h/τ), (h/τ)⁴/120 = {truncation:.1e}, is four orders \
         smaller and left out."
    );
    Ok(Check::new(
        "Lag tier time constant, step to half speed",
        tau,
        measured,
        "s",
    )
    .relative()
    .with_tolerance(tolerance_from(estimate), why))
}

/// Ticks until the ESC's duty cycle first moves after a step of command.
fn ticks_to_move(mut state: EscState, command: f64) -> u32 {
    let start = state.duty();
    (0..1000)
        .position(|_| state.update(command).to_bits() != start.to_bits())
        .and_then(|ticks| u32::try_from(ticks).ok())
        .unwrap_or(u32::MAX)
}

/// Ticks until the ESC's duty cycle reaches `command`.
fn ticks_to_reach(mut state: EscState, command: f64) -> u32 {
    (1..=1000)
        .find(|_| state.update(command) >= command)
        .unwrap_or(u32::MAX)
}

/// The ESC's latency and slew limit, counted in ticks, Eq. (MOT-9).
fn esc_timing() -> Result<Vec<Check>, MotorStudyError> {
    let drive = Drive {
        braking: Braking::Active,
        current_limit_a: None,
    };
    let exact = "Exact: a latency is rounded to the nearest whole tick, halves up, and the delay \
                 line holds that many commands; a slew limit moves the duty cycle by at most \
                 slew/rate a tick and lands on the target.";
    let mut checks = Vec::new();
    for (latency_ms, ticks) in [(2.0, 2), (2.5, 3)] {
        let esc = Esc::new(latency_ms * 1e-3, None, drive, PHYSICS_HZ)?;
        let name = format!("ESC latency {latency_ms} ms at 1 kHz, in ticks");
        let measured = ticks_to_move(esc.start(0.0), 1.0);
        checks.push(
            Check::count(&name, ticks, measured, "ticks").with_tolerance(0.0, exact.to_owned()),
        );
    }
    let slewed = Esc::new(0.0, Some(40.0), drive, PHYSICS_HZ)?;
    let measured = ticks_to_reach(slewed.start(0.0), 1.0);
    checks.push(
        Check::count(
            "ESC slew 40 /s, ticks from 0 to full duty",
            25,
            measured,
            "ticks",
        )
        .with_tolerance(0.0, exact.to_owned()),
    );
    Ok(checks)
}

/// Coasting down without braking under a constant load, Eq. (MOT-11).
fn coast_down(motor: DcMotor) -> Result<Check, MotorStudyError> {
    let load = 0.02;
    let esc = ideal_esc(Braking::None, None)?;
    let start = Start {
        speed_rad_s: 1000.0,
        duty: 0.0,
        command: 0.0,
    };
    let mut world = bench(
        MotorModel::Dc(motor),
        ConstantLoad { torque_n_m: load },
        &esc,
        start,
    )?;
    let steps = 500;
    let simulated = speed_after(&mut world, steps);
    let t = count(steps) * STEP_S;
    let friction = motor.no_load_current_a() / motor.kv_rad_s_v();
    let closed = 1000.0 - (friction + load) * t / motor.inertia_kg_m2();
    let estimate = count(steps) * EPSILON * 1000.0;
    let why = format!(
        "With no braking the current stays at zero, so the deceleration is constant and RK4 \
         integrates it exactly: only rounding, N·ε·ω₀ = {estimate:.1e} rad/s."
    );
    Ok(Check::new(
        "Coast-down without braking, 0.5 s",
        closed,
        simulated,
        "rad/s",
    )
    .with_tolerance(tolerance_from(estimate), why))
}

/// Active braking at zero duty under a constant load, Eq. (MOT-12).
fn active_braking(motor: DcMotor) -> Result<Check, MotorStudyError> {
    let load = 0.02;
    let esc = ideal_esc(Braking::Active, None)?;
    let start = Start {
        speed_rad_s: 1000.0,
        duty: 0.0,
        command: 0.0,
    };
    let mut world = bench(
        MotorModel::Dc(motor),
        ConstantLoad { torque_n_m: load },
        &esc,
        start,
    )?;
    let steps = 100;
    let simulated = speed_after(&mut world, steps);
    let tau = motor.time_constant_s(0.0);
    let floor =
        (motor.no_load_current_a() / motor.kv_rad_s_v() + load) * tau / motor.inertia_kg_m2();
    let t = count(steps) * STEP_S;
    let closed = (1000.0 + floor) * libm::exp(-t / tau) - floor;
    let z = STEP_S / tau;
    let estimate = count(steps) * z * z * z * z * z / 120.0 * (1000.0 + floor);
    let rounding = count(steps) * EPSILON * 1000.0;
    let why = format!(
        "RK4's truncation of e^(−h/τ), N·(h/τ)⁵/120 of the {:.0} rad/s decaying part: \
         {estimate:.1e} rad/s. Rounding, N·ε·ω₀ = {rounding:.1e}, is four orders smaller and \
         left out.",
        1000.0 + floor
    );
    Ok(Check::new(
        "Active braking at zero duty, 0.1 s",
        closed,
        simulated,
        "rad/s",
    )
    .with_tolerance(tolerance_from(estimate), why))
}

/// A start from rest held at a 15 A current limit, Eq. (MOT-13).
fn current_limited_start(motor: DcMotor) -> Result<Check, MotorStudyError> {
    let limit = 15.0;
    let esc = ideal_esc(Braking::Active, Some(limit))?;
    let start = Start {
        speed_rad_s: 0.0,
        duty: 1.0,
        command: 1.0,
    };
    let mut world = bench(
        MotorModel::Dc(motor),
        ConstantLoad { torque_n_m: 0.0 },
        &esc,
        start,
    )?;
    let steps = 50;
    let simulated = speed_after(&mut world, steps);
    let t = count(steps) * STEP_S;
    let closed =
        (limit - motor.no_load_current_a()) * t / (motor.kv_rad_s_v() * motor.inertia_kg_m2());
    let estimate = count(steps) * EPSILON * closed;
    let why = format!(
        "The current stays at its limit, so the acceleration is constant and RK4 integrates it \
         exactly: only rounding, N·ε·ω = {estimate:.1e} rad/s."
    );
    Ok(Check::new(
        "Current-limited start at 15 A, 50 ms",
        closed,
        simulated,
        "rad/s",
    )
    .with_tolerance(tolerance_from(estimate), why))
}

/// Electrical power against mechanical power and losses at the steady 0.05 N·m point,
/// Eq. (MOT-10).
fn power_balance(motor: DcMotor) -> Check {
    let speed = motor.steady_speed(SUPPLY_V, 0.05);
    let input = DriveInput {
        duty: 1.0,
        supply_v: SUPPLY_V,
        drive: Drive {
            braking: Braking::Active,
            current_limit_a: None,
        },
    };
    let point = motor.operating_point(speed, &input);
    let losses = point.current_a * point.current_a * motor.resistance_ohm()
        + motor.no_load_current_a() * speed / motor.kv_rad_s_v();
    let estimate = 10.0 * EPSILON;
    let why = format!(
        "Relative: V·i = τ·ω + i²R + i₀·ω/K_v holds exactly in real numbers; a few roundings, \
         10ε = {estimate:.1e}."
    );
    Check::new(
        "Power balance at the steady 0.05 N·m point",
        point.electrical_power_w,
        point.mechanical_power_w + losses,
        "W",
    )
    .relative()
    .with_tolerance(tolerance_from(estimate), why)
}

/// Every check of the study.
///
/// # Errors
///
/// Only if the test motor or an ESC were not usable, which tests rule out.
pub fn motor_study() -> Result<Vec<Check>, MotorStudyError> {
    let motor = test_motor()?;
    let mut checks = steady_speeds(motor)?;
    checks.push(propeller_steady_speed(motor)?);
    checks.push(small_signal_time_constant(motor)?);
    checks.push(lag_time_constant()?);
    checks.extend(esc_timing()?);
    checks.push(coast_down(motor)?);
    checks.push(active_braking(motor)?);
    checks.push(current_limited_start(motor)?);
    checks.push(power_balance(motor));
    Ok(checks)
}

/// True if every check passed.
#[must_use]
pub fn passed(checks: &[Check]) -> bool {
    checks.iter().all(Check::passed)
}
