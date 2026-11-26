//! The engine error model.

use std::fmt;

use crate::FieldPath;

/// Every error the engine reports.
///
/// Engine crates perform no I/O, so this type has no I/O variant. Front ends read files and
/// wrap their own I/O errors alongside this one.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub enum EngineError {
    /// A configuration document is malformed, uses an unsupported schema, or describes
    /// something physically implausible.
    Config {
        /// Name of the document, usually its file path, as given by the caller.
        document: String,
        /// The value at fault; the whole document for syntax errors.
        path: FieldPath,
        /// What is wrong and, where possible, how to fix it.
        reason: String,
    },
    /// A computed quantity became non-finite or left its physical range.
    Numerical {
        /// Simulation time at which it happened, or `None` outside a running simulation
        /// (for example during trim or analysis).
        sim_time_s: Option<f64>,
        /// Name of the quantity, such as `"rotor 3 inflow"`.
        quantity: &'static str,
        /// The offending value.
        value: f64,
    },
    /// An iterative solver did not converge within its iteration bound.
    Convergence {
        /// Name of the solver, such as `"BEMT inflow"`.
        solver: &'static str,
        /// Iterations performed.
        iterations: u32,
        /// Residual when it stopped.
        residual: f64,
    },
}

impl EngineError {
    /// A [`EngineError::Config`] error.
    #[must_use]
    pub fn config(document: &str, path: FieldPath, reason: impl Into<String>) -> Self {
        Self::Config {
            document: document.to_owned(),
            path,
            reason: reason.into(),
        }
    }
}

impl fmt::Display for EngineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Config {
                document,
                path,
                reason,
            } => write!(f, "{document}: {path}: {reason}"),
            Self::Numerical {
                sim_time_s: Some(time),
                quantity,
                value,
            } => write!(f, "numerical failure at t = {time} s: {quantity} = {value}"),
            Self::Numerical {
                sim_time_s: None,
                quantity,
                value,
            } => write!(f, "numerical failure: {quantity} = {value}"),
            Self::Convergence {
                solver,
                iterations,
                residual,
            } => write!(
                f,
                "{solver} did not converge in {iterations} iterations (residual {residual:e})"
            ),
        }
    }
}

impl std::error::Error for EngineError {}

#[cfg(test)]
mod tests {
    use super::EngineError;
    use crate::FieldPath;

    #[test]
    fn config_errors_name_document_and_field() {
        let path = FieldPath::root().key("mass").key("mass_kg");
        let error = EngineError::config("quad.toml", path, "must be positive; found -1.5");
        assert_eq!(
            error.to_string(),
            "quad.toml: mass.mass_kg: must be positive; found -1.5"
        );
    }

    #[test]
    fn numerical_and_convergence_messages() {
        let numerical = EngineError::Numerical {
            sim_time_s: Some(12.5),
            quantity: "rotor 3 inflow",
            value: f64::NAN,
        };
        assert_eq!(
            numerical.to_string(),
            "numerical failure at t = 12.5 s: rotor 3 inflow = NaN"
        );
        let offline = EngineError::Numerical {
            sim_time_s: None,
            quantity: "trim residual",
            value: f64::INFINITY,
        };
        assert_eq!(
            offline.to_string(),
            "numerical failure: trim residual = inf"
        );
        let convergence = EngineError::Convergence {
            solver: "BEMT inflow",
            iterations: 50,
            residual: 3.2e-4,
        };
        assert_eq!(
            convergence.to_string(),
            "BEMT inflow did not converge in 50 iterations (residual 3.2e-4)"
        );
    }
}
