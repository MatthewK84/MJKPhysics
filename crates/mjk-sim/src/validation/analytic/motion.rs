//! The physics checks: free fall, linear drag, axisymmetric precession and the intermediate-axis
//! flip, each run through the simulation loop and compared with its closed form.

use mjk_dynamics::IntegratorKind;
use mjk_dynamics::mass::MassProperties;
use mjk_env::G0_M_S2;
use mjk_math::{Body, Mat3, UnitQuat, Vec3, World};

use super::bodies::{
    TorqueFree, Unwrapped, Weight, WeightAndDrag, angle_between, run, simulation, start,
};
use super::{AnalyticError, Check, PHYSICS_HZ, tolerance_from};
use crate::validation::analytic::elliptic::complete_elliptic_k;

/// The physics step, s.
const STEP_S: f64 = 1.0 / 1000.0;

/// `f64::EPSILON`: the spacing of doubles near one, a bound on each operation's relative rounding.
const EPSILON: f64 = f64::EPSILON;

/// `steps` as an `f64`.
fn count(steps: u32) -> f64 {
    f64::from(steps)
}

/// The phase error RK4-type methods make over `steps` steps of a motion at `rate_rad_s`:
/// `(h·ω)⁵/120` per step, the first term RK4 drops from `exp(i·h·ω)`.
fn phase_error_rad(steps: u32, rate_rad_s: f64) -> f64 {
    let x = STEP_S * rate_rad_s;
    count(steps) * x * x * x * x * x / 120.0
}

// ---------------------------------------------------------------- free fall

/// Free fall from rest for 10 s, Eq. (VAL-1).
///
/// # Errors
///
/// Only if the test body were not physical or the rate invalid, which tests rule out.
pub fn free_fall(method: IntegratorKind) -> Result<[Check; 2], AnalyticError> {
    let steps = 10_000;
    let props = MassProperties::new(2.0, Vec3::zero(), Mat3::from_diagonal([0.02, 0.03, 0.04]))?;
    let mut sim = simulation(
        Weight,
        props,
        start(Vec3::zero(), Vec3::zero()),
        PHYSICS_HZ,
        method,
    )?;
    run(&mut sim, steps);
    let t = sim.clock().time_s();
    let y = sim.current();
    let (distance, speed) = (0.5 * G0_M_S2 * t * t, G0_M_S2 * t);
    let distance_estimate = count(steps) * EPSILON * distance;
    let speed_estimate = count(steps) * EPSILON * speed;
    let why = |estimate: f64, quantity: &str| {
        format!(
            "RKMK4 integrates translation as RK4 does, and RK4's stages integrate a constant \
             acceleration exactly, so only rounding remains: at most N·ε·{quantity} = \
             {estimate:.1e} over N = {steps} steps."
        )
    };
    Ok([
        Check::new(
            "Free fall: distance fallen in 10 s",
            distance,
            y.position.z,
            "m",
        )
        .with_tolerance(
            tolerance_from(distance_estimate),
            why(distance_estimate, "z"),
        ),
        Check::new("Free fall: speed after 10 s", speed, y.velocity.z, "m/s")
            .with_tolerance(tolerance_from(speed_estimate), why(speed_estimate, "v")),
    ])
}

// ---------------------------------------------------------------- linear drag

/// A projectile under gravity and linear drag for 5 s, Eqs. (VAL-2) and (VAL-3).
///
/// # Errors
///
/// Only if the test body were not physical or the rate invalid, which tests rule out.
pub fn projectile_with_drag(method: IntegratorKind) -> Result<[Check; 2], AnalyticError> {
    let (steps, mass_kg, drag_n_s_m) = (5000, 2.0, 0.5);
    let tau_s = mass_kg / drag_n_s_m;
    let launch = Vec3::new(20.0, 0.0, -15.0);
    let props = MassProperties::new(
        mass_kg,
        Vec3::zero(),
        Mat3::from_diagonal([0.02, 0.03, 0.04]),
    )?;
    let loads = WeightAndDrag { drag_n_s_m };
    let mut sim = simulation(
        loads,
        props,
        start(launch, Vec3::zero()),
        PHYSICS_HZ,
        method,
    )?;
    run(&mut sim, steps);
    let t = sim.clock().time_s();
    let terminal = Vec3::new(0.0, 0.0, G0_M_S2 * tau_s);
    let (position, velocity) = drag_closed_form(launch, tau_s, t);
    let y = sim.current();
    let transient_m = (launch - terminal).norm() * tau_s;
    let h = STEP_S / tau_s;
    let truncation = t / tau_s * h * h * h * h / 120.0;
    // Truncation is about 10⁻⁴ of rounding here, so the tolerances come from rounding alone.
    let position_estimate = count(steps) * EPSILON * position.norm();
    let velocity_estimate = count(steps) * EPSILON * velocity.norm();
    let why = format!(
        "RKMK4 integrates translation as RK4 does. Its truncation, (T/τ)(h/τ)⁴/120 = \
         {truncation:.0e} of the {transient_m:.0} m transient, is far below rounding over \
         N = {steps} steps, which sets both tolerances: N·ε·|x| = {position_estimate:.1e} m and \
         N·ε·|v| = {velocity_estimate:.1e} m/s."
    );
    Ok([
        Check::vector("Linear drag: position after 5 s", position, y.position, "m")
            .with_tolerance(tolerance_from(position_estimate), why.clone()),
        Check::vector(
            "Linear drag: velocity after 5 s",
            velocity,
            y.velocity,
            "m/s",
        )
        .with_tolerance(tolerance_from(velocity_estimate), why),
    ])
}

/// Position and velocity under gravity and linear drag with time constant `tau_s`, launched
/// from the origin at `launch`, Eqs. (VAL-2) and (VAL-3).
#[must_use]
pub fn drag_closed_form(launch: Vec3<World>, tau_s: f64, t: f64) -> (Vec3<World>, Vec3<World>) {
    let terminal = Vec3::new(0.0, 0.0, G0_M_S2 * tau_s);
    let decay = libm::exp(-t / tau_s);
    let velocity = terminal + (launch - terminal) * decay;
    let position = terminal * t + (launch - terminal) * (tau_s * (1.0 - decay));
    (position, velocity)
}

// ---------------------------------------------------------------- axisymmetric body

/// A torque-free axisymmetric body for 10 s: its body-frame and space precession rates and its
/// attitude, Eqs. (VAL-4) to (VAL-6).
///
/// # Errors
///
/// Only if the test body were not physical or the rate invalid, which tests rule out.
pub fn axisymmetric_precession(method: IntegratorKind) -> Result<[Check; 3], AnalyticError> {
    let (steps, transverse, axial) = (10_000, 0.02, 0.035);
    let omega = Vec3::<Body>::new(1.0, 0.0, 12.0);
    let props = MassProperties::new(
        1.0,
        Vec3::zero(),
        Mat3::from_diagonal([transverse, transverse, axial]),
    )?;
    let mut sim = simulation(
        TorqueFree,
        props,
        start(Vec3::zero(), omega),
        PHYSICS_HZ,
        method,
    )?;
    let precession = Precession::of(transverse, axial, omega);
    let mut body_phase = Unwrapped::new(libm::atan2(omega.y, omega.x));
    let mut space_phase = Unwrapped::new(precession.space_angle(UnitQuat::identity()));
    for _ in 0..steps {
        sim.step();
        let y = sim.current();
        body_phase.advance(libm::atan2(y.angular_velocity.y, y.angular_velocity.x));
        space_phase.advance(precession.space_angle(attitude(y.attitude)));
    }
    let t = sim.clock().time_s();
    let attitude_error = angle_between(precession.attitude(t), attitude(sim.current().attitude));
    Ok(precession.checks(
        t,
        steps,
        omega.norm(),
        [body_phase, space_phase],
        attitude_error,
    ))
}

/// The simulated attitude as a unit quaternion; a degenerate one, which never occurs, as the
/// identity.
#[must_use]
pub fn attitude(q: mjk_math::Quat) -> UnitQuat {
    UnitQuat::new(q).unwrap_or_else(UnitQuat::identity)
}

/// The closed-form motion of a torque-free axisymmetric body that starts at identity attitude.
pub struct Precession {
    /// The unit angular momentum, world axes, equal to body axes at the start.
    axis: Vec3<World>,
    /// The space precession rate `|L|/I_t`, rad/s.
    space_rate: f64,
    /// The body precession rate `Ω = (I_a − I_t)ω₃/I_t`, rad/s.
    body_rate: f64,
    /// Unit vectors spanning the plane normal to the momentum.
    plane: (Vec3<World>, Vec3<World>),
}

impl Precession {
    /// The motion of a body with transverse and axial moments `transverse` and `axial` that
    /// starts at identity attitude with body rates `omega`, Eq. (VAL-4).
    #[must_use]
    pub fn of(transverse: f64, axial: f64, omega: Vec3<Body>) -> Self {
        // At identity attitude, body and world axes coincide.
        let momentum = Vec3::new(transverse * omega.x, transverse * omega.y, axial * omega.z);
        Self::new(
            momentum,
            transverse,
            (axial - transverse) * omega.z / transverse,
        )
    }

    fn new(momentum: Vec3<World>, transverse: f64, body_rate: f64) -> Self {
        let length = momentum.norm();
        let axis = momentum / length;
        // The symmetry axis, minus its component along the momentum.
        let u = Vec3::new(0.0, 0.0, 1.0) - axis * axis.z;
        let u = u / u.norm();
        Self {
            axis,
            space_rate: length / transverse,
            body_rate,
            plane: (u, axis.cross(u)),
        }
    }

    /// The angle of the body's symmetry axis around the momentum.
    fn space_angle(&self, attitude: UnitQuat) -> f64 {
        let e3 = Vec3::<World>::from_array(attitude.rotate_array([0.0, 0.0, 1.0]));
        libm::atan2(e3.dot(self.plane.1), e3.dot(self.plane.0))
    }

    /// The space precession rate `|L|/I_t`, rad/s.
    #[must_use]
    pub const fn space_rate(&self) -> f64 {
        self.space_rate
    }

    /// `Rot(L̂, |L|t/I_t) · Rot(e₃, −Ω t)`, Eq. (VAL-6).
    #[must_use]
    pub fn attitude(&self, t: f64) -> UnitQuat {
        let space = self.axis * self.space_rate * t;
        UnitQuat::exp(space.to_array()) * UnitQuat::exp([0.0, 0.0, -self.body_rate * t])
    }

    fn checks(
        &self,
        t: f64,
        steps: u32,
        speed: f64,
        phases: [Unwrapped; 2],
        error: f64,
    ) -> [Check; 3] {
        let body_estimate = phase_error_rad(steps, self.body_rate) / t;
        let attitude_estimate =
            phase_error_rad(steps, speed) + count(steps) * EPSILON * self.space_rate * t;
        let why = |rate: f64, value: f64, per_time: bool| {
            if per_time {
                format!(
                    "RK4-type phase error over the run, N·(h·ω)⁵/120 divided by T = {t:.0} s, \
                     with ω = {rate:.2} rad/s: {value:.1e} rad/s."
                )
            } else {
                format!(
                    "RK4-type phase error, N·(h·ω)⁵/120 with ω = {rate:.2} rad/s, plus rounding: \
                     {value:.1e} rad."
                )
            }
        };
        [
            Check::new(
                "Axisymmetric body: precession rate in body axes",
                self.body_rate,
                phases[0].total_rad() / t,
                "rad/s",
            )
            .with_tolerance(
                tolerance_from(body_estimate),
                why(self.body_rate, body_estimate, true),
            ),
            Check::new(
                "Axisymmetric body: precession rate in space",
                self.space_rate,
                phases[1].total_rad() / t,
                "rad/s",
            )
            .with_tolerance(
                tolerance_from(attitude_estimate / t),
                why(speed, attitude_estimate / t, true),
            ),
            Check::angle("Axisymmetric body: attitude after 10 s", error).with_tolerance(
                tolerance_from(attitude_estimate),
                why(speed, attitude_estimate, false),
            ),
        ]
    }
}

// ---------------------------------------------------------------- intermediate axis

/// A torque-free body spinning near its intermediate axis for 60 s: the flip interval against
/// Landau's elliptic-integral solution, and conservation of |L| and energy, Eq. (VAL-7).
///
/// # Errors
///
/// Only if the test body were not physical or the rate invalid, which tests rule out.
pub fn intermediate_axis(method: IntegratorKind) -> Result<[Check; 3], AnalyticError> {
    let steps = 60_000;
    let moments = [0.010, 0.020, 0.030];
    let omega = Vec3::<Body>::new(0.05, 3.0, 0.05);
    let props = MassProperties::new(1.0, Vec3::zero(), Mat3::from_diagonal(moments))?;
    let mut sim = simulation(
        TorqueFree,
        props,
        start(Vec3::zero(), omega),
        PHYSICS_HZ,
        method,
    )?;
    let invariants = Invariants::of(moments, omega);
    let mut flips = Flips::default();
    let mut drift = [0.0_f64; 2];
    let mut previous = omega.y;
    for _ in 0..steps {
        sim.step();
        let w = sim.current().angular_velocity;
        flips.observe(sim.clock().time_s(), previous, w.y);
        previous = w.y;
        let now = Invariants::of(moments, w);
        drift[0] = drift[0].max((now.momentum / invariants.momentum - 1.0).abs());
        drift[1] = drift[1].max((now.energy / invariants.energy - 1.0).abs());
    }
    let solution = invariants.solution(moments).unwrap_or(Solution::UNDEFINED);
    Ok(intermediate_checks(
        steps,
        omega.norm(),
        solution,
        flips.mean_interval_s(),
        drift,
    ))
}

/// |L| and twice the kinetic energy.
#[derive(Clone, Copy, Debug)]
struct Invariants {
    momentum: f64,
    energy: f64,
}

impl Invariants {
    fn of(moments: [f64; 3], omega: Vec3<Body>) -> Self {
        let l = Vec3::<Body>::new(
            moments[0] * omega.x,
            moments[1] * omega.y,
            moments[2] * omega.z,
        );
        Self {
            momentum: l.norm(),
            energy: l.dot(omega),
        }
    }

    /// Landau and Lifshitz's solution (§37) for `M² > 2E·I₂`.
    fn solution(self, [i1, i2, i3]: [f64; 3]) -> Option<Solution> {
        let m2 = self.momentum * self.momentum;
        let rate = ((i3 - i2) * (m2 - self.energy * i1) / (i1 * i2 * i3)).sqrt();
        let k_squared = (i2 - i1) * (self.energy * i3 - m2) / ((i3 - i2) * (m2 - self.energy * i1));
        let k = complete_elliptic_k(k_squared)?;
        let k_prime_squared = 1.0 - k_squared;
        Some(Solution {
            flip_interval_s: 2.0 * k / rate,
            amplification: 1.0 / (2.0 * k * k_prime_squared),
            k_prime_squared,
        })
    }
}

fn intermediate_checks(
    steps: u32,
    speed: f64,
    solution: Solution,
    measured: f64,
    drift: [f64; 2],
) -> [Check; 3] {
    let invariant_estimate = phase_error_rad(steps, speed) + count(steps) * EPSILON;
    // Near the separatrix, k′² is a small difference of the invariants, so an invariant
    // error δ moves the interval by about δ/(2K·k′²).
    let Solution {
        flip_interval_s,
        amplification,
        k_prime_squared,
    } = solution;
    let flip_estimate = amplification * invariant_estimate;
    let why_flip = format!(
        "Near the separatrix (k′² = {k_prime_squared:.1e}) an invariant error δ moves the \
         interval by about δ/(2K·k′²) = {amplification:.0}δ; with δ = \
         {invariant_estimate:.1e}, {flip_estimate:.1e}."
    );
    let why_invariant = format!(
        "RK4-type truncation N·(h·ω)⁵/120 plus rounding N·ε, with ω = {speed:.2} rad/s: \
         {invariant_estimate:.1e}."
    );
    [
        Check::new(
            "Intermediate axis: time between flips",
            flip_interval_s,
            measured,
            "s",
        )
        .relative()
        .with_tolerance(tolerance_from(flip_estimate), why_flip),
        Check::drift("Intermediate axis: angular momentum over 60 s", drift[0])
            .with_tolerance(tolerance_from(invariant_estimate), why_invariant.clone()),
        Check::drift("Intermediate axis: kinetic energy over 60 s", drift[1])
            .with_tolerance(tolerance_from(invariant_estimate), why_invariant),
    ]
}

/// The closed-form flip interval and its sensitivity to errors in the invariants.
#[derive(Clone, Copy, Debug)]
struct Solution {
    /// `2K(k)/ω_p`, Eq. (VAL-7), s.
    flip_interval_s: f64,
    /// `1/(2K·k′²)`: how much a relative invariant error is magnified in the interval.
    amplification: f64,
    /// `k′² = 1 − k²`: the distance from the separatrix.
    k_prime_squared: f64,
}

impl Solution {
    /// What a body off the formula's branch gets: every comparison fails.
    const UNDEFINED: Self = Self {
        flip_interval_s: f64::NAN,
        amplification: f64::NAN,
        k_prime_squared: f64::NAN,
    };
}

/// Sign changes of the intermediate-axis rate, timed by linear interpolation between steps.
#[derive(Clone, Copy, Debug, Default)]
struct Flips {
    first_s: Option<f64>,
    last_s: f64,
    count: u32,
}

impl Flips {
    fn observe(&mut self, t: f64, before: f64, after: f64) {
        if (before < 0.0) == (after < 0.0) {
            return;
        }
        let crossing = t - STEP_S * after / (after - before);
        self.first_s.get_or_insert(crossing);
        self.last_s = crossing;
        self.count += 1;
    }

    /// The mean time between flips, or NaN with fewer than two.
    fn mean_interval_s(&self) -> f64 {
        let first = self.first_s.unwrap_or(f64::NAN);
        (self.last_s - first) / count(self.count.saturating_sub(1))
    }
}
