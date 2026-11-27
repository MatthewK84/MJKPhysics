//! The integrator study of [#12](https://github.com/MatthewK84/MJKPhysics/issues/12): RK4 with
//! and without renormalization against RKMK4, on a torque-free asymmetric body.
//!
//! Every number is computed with bit-exact arithmetic, so the table is identical on every run
//! and platform. Cost is reported as operation counts for the same reason; wall-clock time is
//! measured separately by `mjk validate integrators --timing`.

use std::ops::{Add, Mul};

use mjk_dynamics::mass::{MassError, MassProperties};
use mjk_dynamics::{
    Coordinates, Integrator, LoadModel, Loads, RigidBodyCoordinates, RigidBodyState,
    RigidBodySystem, Rk4, Rkmk4, System,
};
use mjk_math::{Mat3, Rotation, Vec3, World};

/// Step sizes compared, with the number of steps that covers 100 s.
pub const RUNS: [(f64, u32); 3] = [(0.001, 100_000), (0.01, 10_000), (0.05, 2_000)];

/// The methods compared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    /// RK4 on the quaternion components with no renormalization: what renormalization fixes.
    Rk4Unnormalized,
    /// RK4 on the quaternion components, renormalized after every step.
    Rk4,
    /// Runge–Kutta–Munthe-Kaas, order four: steps on SO(3), never renormalized.
    Rkmk4,
}

impl Method {
    /// Every method, in table order.
    pub const ALL: [Self; 3] = [Self::Rk4Unnormalized, Self::Rk4, Self::Rkmk4];

    /// Name used in the table.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Rk4Unnormalized => "RK4, not renormalized",
            Self::Rk4 => "RK4, renormalized",
            Self::Rkmk4 => "RKMK4",
        }
    }

    /// Rate evaluations, exponential maps and inverse right Jacobians per step.
    #[must_use]
    pub const fn cost(self) -> [u32; 3] {
        match self {
            Self::Rk4Unnormalized | Self::Rk4 => [4, 0, 0],
            // Four stages, each at q₀ ⊗ Exp(θ) with J_r⁻¹(θ), then the final Exp.
            Self::Rkmk4 => [4, 5, 4],
        }
    }

    /// Advances the study body `steps` steps of `dt_s` from its initial state.
    ///
    /// # Errors
    ///
    /// Only if the study body's mass properties were not physical, which a test rules out.
    pub fn advance(self, dt_s: f64, steps: u32) -> Result<RigidBodyCoordinates, MassError> {
        Ok(self.run(dt_s, steps)?.0)
    }

    /// The final coordinates and the largest drifts seen on the way.
    fn run(self, dt_s: f64, steps: u32) -> Result<(RigidBodyCoordinates, Drift), MassError> {
        let system = RigidBodySystem {
            props: study_body()?,
            loads: TorqueFree,
        };
        let y0 = initial_state();
        let mut recorder = Recorder::new(&system.props, &y0);
        let end = match self {
            Self::Rk4Unnormalized => {
                let raw = Unnormalized(system);
                integrate(&Rk4, &raw, Raw(y0), (dt_s, steps), |y| y.0, &mut recorder).0
            }
            Self::Rk4 => integrate(&Rk4, &system, y0, (dt_s, steps), |y| *y, &mut recorder),
            Self::Rkmk4 => integrate(&Rkmk4, &system, y0, (dt_s, steps), |y| *y, &mut recorder),
        };
        Ok((end, recorder.drift))
    }
}

/// One line of the study table.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StudyRow {
    /// The method.
    pub method: Method,
    /// Step size.
    pub dt_s: f64,
    /// Steps taken.
    pub steps: u32,
    /// Largest relative kinetic-energy error over the run.
    pub energy_drift: f64,
    /// Largest relative change of the angular-momentum vector, world axes.
    pub momentum_drift: f64,
    /// Largest deviation of the attitude quaternion's norm from 1.
    pub unit_norm_error: f64,
}

/// Runs every method at every step size.
///
/// # Errors
///
/// Only if the study body's mass properties were not physical, which a test rules out.
pub fn integrator_study() -> Result<Vec<StudyRow>, MassError> {
    let mut rows = Vec::with_capacity(RUNS.len() * Method::ALL.len());
    for (dt_s, steps) in RUNS {
        for method in Method::ALL {
            let (_, drift) = method.run(dt_s, steps)?;
            rows.push(StudyRow {
                method,
                dt_s,
                steps,
                energy_drift: drift.energy,
                momentum_drift: drift.momentum,
                unit_norm_error: drift.unit_norm,
            });
        }
    }
    Ok(rows)
}

/// Header of the study table.
const HEADER: &str = "| Method | h (s) | Steps | Energy drift | Angular-momentum drift \
                      | Unit-norm error | Rate evaluations per step | Exp maps per step \
                      | J_r⁻¹ per step |\n|---|---:|---:|---:|---:|---:|---:|---:|---:|\n";

/// The study table in Markdown, as `mjk validate integrators` prints it and the book includes it.
#[must_use]
pub fn markdown(rows: &[StudyRow]) -> String {
    let lines: String = rows.iter().map(table_line).collect();
    format!("{HEADER}{lines}")
}

fn table_line(row: &StudyRow) -> String {
    let [rates, exps, jacobians] = row.method.cost();
    format!(
        "| {} | {} | {} | {:.1e} | {:.1e} | {:.1e} | {rates} | {exps} | {jacobians} |\n",
        row.method.name(),
        row.dt_s,
        row.steps,
        row.energy_drift,
        row.momentum_drift,
        row.unit_norm_error,
    )
}

// ---------------------------------------------------------------- the study body

#[derive(Clone, Copy, Debug)]
struct TorqueFree;

impl LoadModel for TorqueFree {
    fn loads(&self, _: f64, _: &RigidBodyState, _: &MassProperties) -> Loads {
        Loads::default()
    }
}

/// An X500-class inertia tensor with small products of inertia.
fn study_body() -> Result<MassProperties, MassError> {
    let inertia = Mat3::from_rows([
        [0.021, 0.0004, -0.0006],
        [0.0004, 0.024, 0.0002],
        [-0.0006, 0.0002, 0.040],
    ]);
    MassProperties::new(2.0, Vec3::zero(), inertia)
}

/// Spinning mostly about the intermediate axis, so the body tumbles (the Dzhanibekov effect)
/// and every component of ω changes sign during the run.
fn initial_state() -> RigidBodyCoordinates {
    let mut state = RigidBodyState::at_rest(Vec3::zero(), Rotation::identity());
    state.omega_rad_s = Vec3::new(0.2, 6.0, 0.3);
    RigidBodyCoordinates::from(state)
}

// ---------------------------------------------------------------- measurement

#[derive(Clone, Copy, Debug, Default)]
struct Drift {
    energy: f64,
    momentum: f64,
    unit_norm: f64,
}

/// The conserved quantities at the start, and the largest relative errors seen since.
struct Recorder {
    props: MassProperties,
    energy_j: f64,
    momentum_w: Vec3<World>,
    drift: Drift,
}

impl Recorder {
    fn new(props: &MassProperties, y0: &RigidBodyCoordinates) -> Self {
        let state = y0.to_state();
        Self {
            props: *props,
            energy_j: state.map_or(f64::NAN, |s| s.kinetic_energy_j(props)),
            momentum_w: state.map_or(Vec3::zero(), |s| s.angular_momentum_w(props)),
            drift: Drift::default(),
        }
    }

    fn record(&mut self, y: &RigidBodyCoordinates) {
        let norm_error = (y.attitude.norm() - 1.0).abs();
        let (energy, momentum) = y.to_state().map_or((f64::NAN, f64::NAN), |s| {
            let energy = (s.kinetic_energy_j(&self.props) - self.energy_j).abs() / self.energy_j;
            let change = s.angular_momentum_w(&self.props) - self.momentum_w;
            (energy, change.norm() / self.momentum_w.norm())
        });
        self.drift.energy = self.drift.energy.max(energy);
        self.drift.momentum = self.drift.momentum.max(momentum);
        self.drift.unit_norm = self.drift.unit_norm.max(norm_error);
    }
}

/// Takes `steps` steps of `dt_s`, recording the drifts after each one.
fn integrate<S: System, I: Integrator<S>>(
    method: &I,
    system: &S,
    y0: S::Coordinates,
    (dt_s, steps): (f64, u32),
    view: impl Fn(&S::Coordinates) -> RigidBodyCoordinates,
    recorder: &mut Recorder,
) -> S::Coordinates {
    let mut y = y0;
    for n in 0..steps {
        y = method.step(system, f64::from(n) * dt_s, &y, dt_s);
        recorder.record(&view(&y));
    }
    y
}

// ---------------------------------------------------------------- RK4 without renormalization

/// The rigid body with a projection that does nothing, so RK4's drift off the unit sphere shows.
struct Unnormalized<L: LoadModel>(RigidBodySystem<L>);

#[derive(Clone, Copy, Debug)]
struct Raw(RigidBodyCoordinates);

impl Add for Raw {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl Mul<f64> for Raw {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self(self.0 * rhs)
    }
}

impl Coordinates for Raw {
    fn positions(self) -> Self {
        Self(self.0.positions())
    }
    fn velocities(self) -> Self {
        Self(self.0.velocities())
    }
    fn project(self) -> Self {
        self
    }
}

impl<L: LoadModel> System for Unnormalized<L> {
    type Coordinates = Raw;
    fn rate(&self, time_s: f64, y: &Raw) -> Raw {
        Raw(self.0.rate(time_s, &y.0))
    }
}
