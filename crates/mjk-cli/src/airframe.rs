//! `mjk airframe`: loads an airframe file and prints its generated page
//! ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)).
//!
//! Warnings, such as a number without a source, go to standard error; the file still loads.

use mjk_sim::config::{airframe_page, load_airframe};

use crate::exit::{Exit, Failure};
use crate::files::{self, STDIO};

/// Loads the airframe at `path`, or on standard input for `-`, and prints its page.
///
/// # Errors
///
/// An I/O failure if the file cannot be read, and an input failure naming the field at fault if
/// it does not load.
pub fn run(path: &str) -> Result<Exit, Failure> {
    let document = files::input_name(path);
    let text = files::read(path)?;
    let loaded = load_airframe(document, &text).map_err(|error| Failure::engine(&error))?;
    for warning in &loaded.warnings {
        eprintln!("warning: {document}: {warning}");
    }
    files::write(STDIO, &airframe_page(&loaded))?;
    Ok(Exit::Success)
}
