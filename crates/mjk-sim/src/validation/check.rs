//! Checks against closed forms, shared by the validation studies
//! ([#16](https://github.com/MatthewK84/MJKPhysics/issues/16),
//! [#19](https://github.com/MatthewK84/MJKPhysics/issues/19)).
//!
//! A [`Check`] compares a simulated value with its closed form. Each tolerance comes from an
//! error estimate computed before the run, never from the result: [`tolerance_from`] is ten
//! times the estimate, rounded up to a power of ten, and every check states its estimate in the
//! published table.

use mjk_math::{Frame, Vec3};

/// How a check's error is measured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Measure {
    /// An absolute difference in the given unit.
    Absolute(&'static str),
    /// The distance between two vectors, in the given unit; the table shows their magnitudes.
    Vector(&'static str),
    /// A difference of whole numbers, in the given unit.
    Count(&'static str),
    /// A difference relative to the closed form.
    Relative,
    /// A difference between measured and nominal convergence orders.
    Order,
}

/// One comparison with a closed form.
#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    /// What is compared.
    pub name: String,
    /// The closed-form value, as published.
    pub closed_form: String,
    /// The simulated value, as published.
    pub simulated: String,
    /// The error.
    pub error: f64,
    /// How the error is measured.
    pub measure: Measure,
    /// The largest error that passes.
    pub tolerance: f64,
    /// The error estimate behind the tolerance.
    pub why: String,
}

impl Check {
    /// A scalar compared with its closed form, by absolute difference.
    #[must_use]
    pub fn new(name: &str, closed_form: f64, simulated: f64, unit: &'static str) -> Self {
        Self::raw(
            name,
            [ten_figures(closed_form), ten_figures(simulated)],
            (simulated - closed_form).abs(),
            Measure::Absolute(unit),
        )
    }

    /// A vector compared with its closed form: the table shows magnitudes, the error is the
    /// distance between the vectors.
    #[must_use]
    pub fn vector<F: Frame>(
        name: &str,
        closed_form: Vec3<F>,
        simulated: Vec3<F>,
        unit: &'static str,
    ) -> Self {
        Self::raw(
            name,
            [
                ten_figures(closed_form.norm()),
                ten_figures(simulated.norm()),
            ],
            (simulated - closed_form).norm(),
            Measure::Vector(unit),
        )
    }

    /// An attitude compared with its closed form: the error is the angle between them.
    #[must_use]
    pub fn angle(name: &str, error_rad: f64) -> Self {
        Self::raw(
            name,
            ["R(t), Eq. (VAL-6)".to_owned(), "angle from it".to_owned()],
            error_rad,
            Measure::Absolute("rad"),
        )
    }

    /// A conserved quantity: the error is its largest relative change.
    #[must_use]
    pub fn drift(name: &str, drift: f64) -> Self {
        Self::raw(
            name,
            ["constant".to_owned(), "largest change".to_owned()],
            drift,
            Measure::Relative,
        )
    }

    /// A measured convergence order against the method's nominal one.
    #[must_use]
    pub fn order(name: String, nominal: f64, measured: f64) -> Self {
        let mut check = Self::raw(
            "",
            [format!("{nominal:.0}"), format!("{measured:.2}")],
            (measured - nominal).abs(),
            Measure::Order,
        );
        check.name = name;
        check
    }

    /// A whole number of steps compared with its closed form, such as an ESC's latency in ticks.
    #[must_use]
    pub fn count(name: &str, closed_form: u32, simulated: u32, unit: &'static str) -> Self {
        Self::raw(
            name,
            [closed_form.to_string(), simulated.to_string()],
            f64::from(closed_form.abs_diff(simulated)),
            Measure::Count(unit),
        )
    }

    fn raw(
        name: &str,
        [closed_form, simulated]: [String; 2],
        error: f64,
        measure: Measure,
    ) -> Self {
        Self {
            name: name.to_owned(),
            closed_form,
            simulated,
            error,
            measure,
            tolerance: f64::NAN,
            why: String::new(),
        }
    }

    /// The same check, measuring its error relative to the closed form.
    #[must_use]
    pub fn relative(mut self) -> Self {
        let closed_form: f64 = self.closed_form.parse().unwrap_or(f64::NAN);
        self.error /= closed_form.abs();
        self.measure = Measure::Relative;
        self
    }

    /// The same check with its tolerance and the estimate behind it.
    #[must_use]
    pub fn with_tolerance(mut self, tolerance: f64, why: String) -> Self {
        self.tolerance = tolerance;
        self.why = why;
        self
    }

    /// True if the error is within the tolerance. A NaN error never passes.
    #[must_use]
    pub fn passed(&self) -> bool {
        self.error <= self.tolerance
    }
}

/// Ten times `estimate`, rounded up to a power of ten.
#[must_use]
pub fn tolerance_from(estimate: f64) -> f64 {
    libm::pow(10.0, libm::ceil(libm::log10(10.0 * estimate)))
}

/// `value` with ten significant figures, for values from 10⁻¹ to 10³.
fn ten_figures(value: f64) -> String {
    let magnitude = value.abs();
    let decimals = if magnitude >= 100.0 {
        7
    } else if magnitude >= 10.0 {
        8
    } else if magnitude >= 1.0 {
        9
    } else {
        10
    };
    format!("{value:.decimals$}")
}

/// Header of the study table.
const HEADER: &str = "| # | Check | Closed form | Simulated | Error | Tolerance | Result |\n\
                      |---:|---|---:|---:|---:|---:|---|\n";

/// The study as Markdown: a table of every check, then the estimate behind each tolerance.
#[must_use]
pub fn markdown(checks: &[Check]) -> String {
    let rows: String = checks
        .iter()
        .zip(1..)
        .map(|(check, number)| table_line(number, check))
        .collect();
    let reasons: String = reasons(checks)
        .iter()
        .map(|(first, last, why)| {
            if first == last {
                format!("- **{first}.** {why}\n")
            } else {
                format!("- **{first}–{last}.** {why}\n")
            }
        })
        .collect();
    let vectors = checks
        .iter()
        .any(|check| matches!(check.measure, Measure::Vector(_)));
    let note = if vectors {
        "\n*Vector checks show magnitudes; their error is the distance between the vectors.*\n"
    } else {
        ""
    };
    format!("{HEADER}{rows}{note}\nWhy each tolerance:\n\n{reasons}")
}

/// The reasons behind the tolerances, with runs of checks that share one merged:
/// (first check, last check, reason).
fn reasons(checks: &[Check]) -> Vec<(u32, u32, &str)> {
    let mut groups: Vec<(u32, u32, &str)> = Vec::new();
    for (check, number) in checks.iter().zip(1..) {
        match groups.last_mut() {
            Some((_, last, why)) if *why == check.why => *last = number,
            _ => groups.push((number, number, check.why.as_str())),
        }
    }
    groups
}

fn table_line(number: u32, check: &Check) -> String {
    let (error, tolerance) = match check.measure {
        Measure::Absolute(unit) | Measure::Vector(unit) => (
            format!("{:.1e} {unit}", check.error),
            format!("{:.0e} {unit}", check.tolerance),
        ),
        Measure::Count(unit) => (
            format!("{:.0} {unit}", check.error),
            format!("{:.0} {unit}", check.tolerance),
        ),
        Measure::Relative => (
            format!("{:.1e} (relative)", check.error),
            format!("{:.0e}", check.tolerance),
        ),
        Measure::Order => (
            format!("{:.2}", check.error),
            format!("{:.1}", check.tolerance),
        ),
    };
    let result = if check.passed() { "Pass" } else { "Fail" };
    format!(
        "| {number} | {} | {} | {} | {error} | {tolerance} | {result} |\n",
        check.name, check.closed_form, check.simulated
    )
}
