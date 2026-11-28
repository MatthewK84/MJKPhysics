//! Reading and writing files, with `-` for the standard streams.

use std::fs;
use std::io::{self, Read, Write};
use std::path::Path;

use crate::exit::{Exit, Failure};

/// The path that means standard input or standard output.
pub const STDIO: &str = "-";

/// How a path is named in messages.
#[must_use]
pub fn input_name(path: &str) -> &str {
    if path == STDIO {
        "standard input"
    } else {
        path
    }
}

/// The whole of `path` as text, or of standard input for `-`.
///
/// # Errors
///
/// An [`Exit::Io`] failure naming the file.
pub fn read(path: &str) -> Result<String, Failure> {
    let result = if path == STDIO {
        let mut text = String::new();
        io::stdin().read_to_string(&mut text).map(|_| text)
    } else {
        fs::read_to_string(path)
    };
    result.map_err(|error| {
        Failure::new(
            Exit::Io,
            format!("cannot read {}: {error}", input_name(path)),
        )
    })
}

/// Writes `text` to `path`, or to standard output for `-`.
///
/// # Errors
///
/// An [`Exit::Io`] failure naming the file.
pub fn write(path: &str, text: &str) -> Result<(), Failure> {
    let result = if path == STDIO {
        let mut stdout = io::stdout().lock();
        stdout
            .write_all(text.as_bytes())
            .and_then(|()| stdout.flush())
    } else {
        fs::write(path, text)
    };
    let name = if path == STDIO {
        "standard output"
    } else {
        path
    };
    result.map_err(|error| Failure::new(Exit::Io, format!("cannot write {name}: {error}")))
}

/// Creates directory `dir` and any missing parents.
///
/// # Errors
///
/// An [`Exit::Io`] failure naming the directory.
pub fn create_dir(dir: &Path) -> Result<(), Failure> {
    fs::create_dir_all(dir).map_err(|error| {
        Failure::new(
            Exit::Io,
            format!("cannot create {}: {error}", dir.display()),
        )
    })
}
