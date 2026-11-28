//! Whole-airframe checks ([#18](https://github.com/MatthewK84/MJKPhysics/issues/18)): enough
//! thrust to fly, and rotor disks that do not overlap.
//!
//! Two rotors whose hubs are at least the sum of their radii apart are clear. Closer than that,
//! parallel rotors whose disk planes are more than [`COPLANAR_M`] apart are stacked, as in a
//! coaxial pair, and pass; in the same plane their disks overlap. Tilted rotors closer than the
//! sum of their radii are refused, because this version cannot verify their clearance.

use mjk_core::{EngineError, FieldPath};
use mjk_env::G0_M_S2;

use super::airframe::Airframe;
use super::parameter::{Resolver, significant};
use super::rotors::{Propeller, Rotor};

/// Disk planes closer than this are treated as one plane, m.
pub const COPLANAR_M: f64 = 1e-3;

/// Axes whose cross product is shorter than this are parallel: about 0.2 arcseconds.
const PARALLEL: f64 = 1e-6;

/// The sum of the rotors' largest static thrusts over the airframe's weight at standard gravity.
#[must_use]
pub fn thrust_to_weight(airframe: &Airframe) -> f64 {
    total_thrust_n(airframe) / weight_n(airframe)
}

fn total_thrust_n(airframe: &Airframe) -> f64 {
    airframe
        .rotors
        .iter()
        .filter_map(|rotor| airframe.propellers.get(&rotor.propeller))
        .map(|propeller| propeller.max_thrust_n)
        .sum()
}

fn weight_n(airframe: &Airframe) -> f64 {
    airframe.mass.mass_kg() * G0_M_S2
}

/// The airframe has rotors, can lift itself, and its rotor disks do not overlap.
///
/// # Errors
///
/// [`EngineError::Config`] at `rotors` for no rotors or a thrust-to-weight ratio of 1 or less,
/// and at the later of two rotors whose disks overlap or cannot be checked.
pub fn check_rotors(resolver: &Resolver<'_>, airframe: &Airframe) -> Result<(), EngineError> {
    let path = FieldPath::root().key("rotors");
    if airframe.rotors.is_empty() {
        return Err(resolver.error(path, "an airframe needs at least one rotor"));
    }
    let ratio = thrust_to_weight(airframe);
    if ratio <= 1.0 {
        let reason = format!(
            "thrust-to-weight is {}: the rotors' {} N of maximum thrust cannot lift {} N",
            significant(ratio),
            significant(total_thrust_n(airframe)),
            significant(weight_n(airframe))
        );
        return Err(resolver.error(path, reason));
    }
    let clash = pairs(airframe.rotors.len()).find_map(|(first, second)| {
        let reason = clash(airframe, first, second)?;
        Some((second, reason))
    });
    clash.map_or(Ok(()), |(index, reason)| {
        Err(resolver.error(path.index(index), reason))
    })
}

/// Every pair of indices below `count`, the first smaller.
fn pairs(count: usize) -> impl Iterator<Item = (usize, usize)> {
    (0..count).flat_map(move |second| (0..second).map(move |first| (first, second)))
}

/// Why rotors `first` and `second` clash, if they do.
fn clash(airframe: &Airframe, first: usize, second: usize) -> Option<String> {
    let (a, b) = (airframe.rotors.get(first)?, airframe.rotors.get(second)?);
    let reach = radius(airframe, a)? + radius(airframe, b)?;
    let offset = b.position_m - a.position_m;
    let distance = offset.norm();
    if distance >= reach {
        return None;
    }
    let apart = format!(
        "their hubs are {} m apart, less than the {} m their radii add up to",
        significant(distance),
        significant(reach)
    );
    if tilted(a, b) {
        return Some(format!(
            "it is tilted to rotors[{first}] and {apart}: this version cannot verify their \
             clearance"
        ));
    }
    let stacked = offset.dot(a.axis).abs() > COPLANAR_M;
    (!stacked).then(|| format!("its disk overlaps rotors[{first}]'s: {apart}"))
}

/// True if the rotors' axes are not parallel.
fn tilted(a: &Rotor, b: &Rotor) -> bool {
    a.axis.cross(b.axis).norm() > PARALLEL
}

fn radius(airframe: &Airframe, rotor: &Rotor) -> Option<f64> {
    airframe
        .propellers
        .get(&rotor.propeller)
        .map(|propeller: &Propeller| 0.5 * propeller.diameter_m)
}
