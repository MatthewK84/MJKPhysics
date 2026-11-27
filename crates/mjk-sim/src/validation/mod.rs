//! Validation studies whose results are published in the book, run by `mjk validate <study>`.
//!
//! Each study returns its results as data and formats them as Markdown; the book includes the
//! generated tables, and a test fails if a committed table no longer matches the code.

pub mod determinism;
pub mod integrators;
