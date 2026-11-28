//! The brushless motor ([#19](https://github.com/MatthewK84/MJKPhysics/issues/19)): a DC-equivalent
//! model, Eqs. (MOT-1) to (MOT-7), and a first-order lag tier, Eq. (MOT-8).
//!
//! The DC-equivalent model treats the motor and its commutation as a DC motor with back-EMF
//! constant `1/K_v`. Winding inductance is neglected, so the current follows the applied voltage
//! at once: L/R is short next to the mechanical time constant of tens of milliseconds. The only
//! state is the rotor's speed, a non-negative magnitude; the direction it turns comes from the
//! airframe.

use std::fmt;

/// What the ESC does when the applied voltage is below the motor's back-EMF.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Braking {
    /// The ESC drives the winding down to the applied voltage, so current can reverse and brake
    /// the motor, returning energy to the supply.
    Active,
    /// The current cannot reverse: the motor coasts, slowed only by its friction and its load.
    None,
}

/// How the ESC drives the motor: its braking, and the largest current it allows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Drive {
    /// Braking.
    pub braking: Braking,
    /// The largest current, A; none means unlimited.
    pub current_limit_a: Option<f64>,
}

/// What the motor sees in one step: the ESC's duty cycle, the supply voltage, and how it drives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveInput {
    /// Duty cycle, from 0 to 1.
    pub duty: f64,
    /// Supply voltage, V.
    pub supply_v: f64,
    /// Braking and current limit.
    pub drive: Drive,
}

impl DriveInput {
    /// The average voltage at the motor's terminals, Eq. (MOT-9).
    #[must_use]
    pub fn voltage_v(&self) -> f64 {
        self.duty * self.supply_v
    }
}

/// Why a motor's parameters are not usable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorError {
    /// The parameter.
    pub name: &'static str,
    /// Its value.
    pub value: f64,
    /// True if zero is allowed.
    pub zero_allowed: bool,
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let rule = if self.zero_allowed {
            "finite and not negative"
        } else {
            "positive and finite"
        };
        write!(f, "{} must be {rule}; found {}", self.name, self.value)
    }
}

fn check(name: &'static str, value: f64, zero_allowed: bool) -> Result<f64, MotorError> {
    let usable = value.is_finite() && (value > 0.0 || (zero_allowed && value >= 0.0));
    if usable {
        return Ok(value);
    }
    Err(MotorError {
        name,
        value,
        zero_allowed,
    })
}

/// A DC-equivalent brushless motor turning a rotor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DcMotor {
    kv_rad_s_v: f64,
    resistance_ohm: f64,
    no_load_current_a: f64,
    inertia_kg_m2: f64,
}

/// The motor's electrical and mechanical state at one instant, Eq. (MOT-10).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OperatingPoint {
    /// Winding current, A; negative while braking.
    pub current_a: f64,
    /// Torque on the rotor, N·m.
    pub torque_n_m: f64,
    /// Power into the winding, V·i, W.
    pub electrical_power_w: f64,
    /// Power to the rotor, τ·ω, W.
    pub mechanical_power_w: f64,
    /// Current drawn from the supply through a lossless ESC, d·i, A.
    pub supply_current_a: f64,
}

impl DcMotor {
    /// A motor with speed constant `kv_rad_s_v`, winding resistance `resistance_ohm`, no-load
    /// current `no_load_current_a`, turning a rotor of inertia `inertia_kg_m2` (motor and
    /// propeller together).
    ///
    /// # Errors
    ///
    /// A [`MotorError`] if the speed constant, resistance or inertia is not positive and finite,
    /// or the no-load current is negative or not finite.
    pub fn new(
        kv_rad_s_v: f64,
        resistance_ohm: f64,
        no_load_current_a: f64,
        inertia_kg_m2: f64,
    ) -> Result<Self, MotorError> {
        Ok(Self {
            kv_rad_s_v: check("the speed constant", kv_rad_s_v, false)?,
            resistance_ohm: check("the resistance", resistance_ohm, false)?,
            no_load_current_a: check("the no-load current", no_load_current_a, true)?,
            inertia_kg_m2: check("the inertia", inertia_kg_m2, false)?,
        })
    }

    /// Speed constant, rad/(s·V).
    #[must_use]
    pub const fn kv_rad_s_v(&self) -> f64 {
        self.kv_rad_s_v
    }

    /// Winding resistance, Ω.
    #[must_use]
    pub const fn resistance_ohm(&self) -> f64 {
        self.resistance_ohm
    }

    /// No-load current, A.
    #[must_use]
    pub const fn no_load_current_a(&self) -> f64 {
        self.no_load_current_a
    }

    /// Inertia of the rotor, kg·m².
    #[must_use]
    pub const fn inertia_kg_m2(&self) -> f64 {
        self.inertia_kg_m2
    }

    /// The winding current at `speed_rad_s` with `voltage_v` applied, Eqs. (MOT-1) and (MOT-2):
    /// the free current, held within the drive's limits. Without active braking it cannot
    /// reverse.
    #[must_use]
    pub fn current_a(&self, speed_rad_s: f64, voltage_v: f64, drive: Drive) -> f64 {
        // Eq. (MOT-1)
        let free = (voltage_v - speed_rad_s / self.kv_rad_s_v) / self.resistance_ohm;
        if free.is_nan() {
            // Seen, not hidden: `min` and `max` would replace it with a limit.
            return free;
        }
        let limit = drive.current_limit_a.unwrap_or(f64::INFINITY);
        let floor = match drive.braking {
            Braking::Active => -limit,
            Braking::None => 0.0,
        };
        // Eq. (MOT-2)
        free.min(limit).max(floor)
    }

    /// The torque on the rotor from `current_a`, Eq. (MOT-3): `K_t = 1/K_v` in SI units, less
    /// the friction the no-load current stands for.
    #[must_use]
    pub fn torque_n_m(&self, current_a: f64) -> f64 {
        (current_a - self.no_load_current_a) / self.kv_rad_s_v
    }

    /// The rotor's angular acceleration, Eq. (MOT-4). At rest, a net torque against the spin
    /// direction holds it at rest rather than reversing it.
    #[must_use]
    pub fn acceleration(
        &self,
        speed_rad_s: f64,
        voltage_v: f64,
        load_n_m: f64,
        drive: Drive,
    ) -> f64 {
        let torque = self.torque_n_m(self.current_a(speed_rad_s, voltage_v, drive));
        let rate = (torque - load_n_m) / self.inertia_kg_m2;
        if speed_rad_s <= 0.0 && rate < 0.0 {
            return 0.0;
        }
        rate
    }

    /// Current, torque and power at `speed_rad_s` with `input` applied, Eq. (MOT-10).
    #[must_use]
    pub fn operating_point(&self, speed_rad_s: f64, input: &DriveInput) -> OperatingPoint {
        let voltage = input.voltage_v();
        let current_a = self.current_a(speed_rad_s, voltage, input.drive);
        let torque_n_m = self.torque_n_m(current_a);
        OperatingPoint {
            current_a,
            torque_n_m,
            electrical_power_w: voltage * current_a,
            mechanical_power_w: torque_n_m * speed_rad_s,
            supply_current_a: input.duty * current_a,
        }
    }

    /// The steady speed under a constant load torque, Eq. (MOT-5), while the current is within
    /// its limits and the speed positive.
    #[must_use]
    pub fn steady_speed(&self, voltage_v: f64, load_n_m: f64) -> f64 {
        let current = self.no_load_current_a + load_n_m * self.kv_rad_s_v;
        self.kv_rad_s_v * (voltage_v - self.resistance_ohm * current)
    }

    /// The steady speed under a propeller-like load `coefficient·ω²`, Eq. (MOT-7), while the
    /// current is within its limits and the speed positive.
    #[must_use]
    pub fn steady_speed_quadratic(&self, voltage_v: f64, coefficient_n_m_s2: f64) -> f64 {
        let damping = self.back_emf_damping();
        let drive = (voltage_v / self.resistance_ohm - self.no_load_current_a) / self.kv_rad_s_v;
        // The root of k·ω² + a·ω − c = 0 written without cancellation; it is c/a when k = 0.
        let discriminant = damping * damping + 4.0 * coefficient_n_m_s2 * drive;
        2.0 * drive / (damping + discriminant.sqrt())
    }

    /// The small-signal time constant of the rotor's speed, Eq. (MOT-6), where the load's
    /// torque grows by `load_slope` per rad/s of speed.
    #[must_use]
    pub fn time_constant_s(&self, load_slope: f64) -> f64 {
        self.inertia_kg_m2 / (self.back_emf_damping() + load_slope)
    }

    /// The torque per rad/s with which back-EMF opposes a change of speed, `1/(R·K_v²)`.
    fn back_emf_damping(&self) -> f64 {
        1.0 / (self.resistance_ohm * self.kv_rad_s_v * self.kv_rad_s_v)
    }
}

/// A motor known only by its speed's time constant, Eq. (MOT-8): for airframes where that is all
/// that is published.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LagMotor {
    time_constant_s: f64,
    max_speed_rad_s: f64,
}

impl LagMotor {
    /// A motor whose speed follows `duty · max_speed_rad_s` with time constant
    /// `time_constant_s`.
    ///
    /// # Errors
    ///
    /// A [`MotorError`] if either is not positive and finite.
    pub fn new(time_constant_s: f64, max_speed_rad_s: f64) -> Result<Self, MotorError> {
        Ok(Self {
            time_constant_s: check("the time constant", time_constant_s, false)?,
            max_speed_rad_s: check("the largest speed", max_speed_rad_s, false)?,
        })
    }

    /// The time constant, s.
    #[must_use]
    pub const fn time_constant_s(&self) -> f64 {
        self.time_constant_s
    }

    /// The speed at full duty, rad/s.
    #[must_use]
    pub const fn max_speed_rad_s(&self) -> f64 {
        self.max_speed_rad_s
    }

    /// The rotor's angular acceleration, Eq. (MOT-8).
    #[must_use]
    pub fn acceleration(&self, speed_rad_s: f64, duty: f64) -> f64 {
        (duty * self.max_speed_rad_s - speed_rad_s) / self.time_constant_s
    }
}

/// A motor of either tier.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotorModel {
    /// The DC-equivalent model.
    Dc(DcMotor),
    /// The first-order lag tier.
    Lag(LagMotor),
}

impl MotorModel {
    /// The rotor's angular acceleration at `speed_rad_s` with `input` applied and load torque
    /// `load_n_m`. The lag tier's time constant already includes its load.
    #[must_use]
    pub fn acceleration(&self, speed_rad_s: f64, input: &DriveInput, load_n_m: f64) -> f64 {
        match self {
            Self::Dc(motor) => {
                motor.acceleration(speed_rad_s, input.voltage_v(), load_n_m, input.drive)
            }
            Self::Lag(motor) => motor.acceleration(speed_rad_s, input.duty),
        }
    }
}
