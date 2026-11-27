//! Gravity (#15): standard gravity, and WGS-84 normal gravity against the exact field of the
//! level ellipsoid.

use mjk_env::gravity::wgs84;
use mjk_env::{Environment, EnvironmentError, G0_M_S2, Gravity, NormalGravity};
use mjk_math::{Vec3, World};

/// Exact normal gravity of the WGS-84 level ellipsoid, from its four defining parameters in
/// ellipsoidal-harmonic coordinates (Heiskanen and Moritz, 1967, eq. 2-151), computed
/// independently in double precision: (latitude in degrees, height in m, gravity in m/s²).
const EXACT: [(f64, f64, f64); 12] = [
    (0.0, -5000.0, 9.795_782_506_053_7),
    (0.0, 0.0, 9.780_325_335_904_06),
    (0.0, 5000.0, 9.764_904_491_302_355),
    (0.0, 20_000.0, 9.718_858_773_096_393),
    (45.0, -5000.0, 9.821_643_914_196_663),
    (45.0, 0.0, 9.806_197_769_377_293),
    (45.0, 5000.0, 9.790_787_898_574_193),
    (45.0, 20_000.0, 9.744_774_796_883_2),
    (90.0, -5000.0, 9.847_620_006_910_715),
    (90.0, 0.0, 9.832_184_937_863_067),
    (90.0, 5000.0, 9.816_786_090_242_056),
    (90.0, 20_000.0, 9.770_805_746_907_133),
];

/// Normal gravity at a latitude in degrees.
fn normal_at(latitude_deg: f64) -> Option<NormalGravity> {
    NormalGravity::new(latitude_deg.to_radians()).ok()
}

/// The derived constants of the level ellipsoid, from the four defining parameters:
/// (e², m, γe, γp, k).
fn derived_constants() -> (f64, f64, f64, f64, f64) {
    let a = wgs84::SEMI_MAJOR_AXIS_M;
    let b = a * (1.0 - wgs84::FLATTENING);
    let linear_eccentricity = (a * a - b * b).sqrt();
    let second_eccentricity = linear_eccentricity / b;
    let x = second_eccentricity;
    let q0 = 0.5 * ((1.0 + 3.0 / (x * x)) * libm::atan(x) - 3.0 / x);
    let q0_prime = 3.0 * (1.0 + 1.0 / (x * x)) * (1.0 - libm::atan(x) / x) - 1.0;
    let omega = wgs84::ANGULAR_VELOCITY_RAD_S;
    let m = omega * omega * a * a * b / wgs84::GM_M3_S2;
    let term = m * second_eccentricity * q0_prime / q0;
    let gamma_e = wgs84::GM_M3_S2 / (a * b) * (1.0 - m - term / 6.0);
    let gamma_p = wgs84::GM_M3_S2 / (a * a) * (1.0 + term / 3.0);
    let e2 = (a * a - b * b) / (a * a);
    (e2, m, gamma_e, gamma_p, b * gamma_p / (a * gamma_e) - 1.0)
}

#[test]
fn the_derived_constants_follow_from_the_defining_parameters() {
    let (e2, m, gamma_e, gamma_p, k) = derived_constants();
    // The constants are the 40-digit values rounded to double precision. Recomputed in double
    // precision, the series behind γe, γp and k lose a few digits to cancellation in q₀.
    assert!((e2 - wgs84::ECCENTRICITY_SQUARED).abs() < 1e-17);
    assert!((m - wgs84::GRAVITY_RATIO_M).abs() < 1e-17);
    assert!((gamma_e - wgs84::EQUATORIAL_GRAVITY_M_S2).abs() < 1e-12);
    assert!((gamma_p - wgs84::POLAR_GRAVITY_M_S2).abs() < 1e-12);
    assert!((k - wgs84::SOMIGLIANA_K).abs() < 1e-13);
}

#[test]
fn somigliana_gives_the_equator_and_pole_values() {
    assert!(
        (normal_at(0.0)
            .expect("a valid latitude")
            .on_ellipsoid_m_s2()
            - wgs84::EQUATORIAL_GRAVITY_M_S2)
            .abs()
            < 1e-15
    );
    assert!(
        (normal_at(90.0)
            .expect("a valid latitude")
            .on_ellipsoid_m_s2()
            - wgs84::POLAR_GRAVITY_M_S2)
            .abs()
            < 1e-9
    );
    assert!(
        (normal_at(-90.0)
            .expect("a valid latitude")
            .on_ellipsoid_m_s2()
            - normal_at(90.0)
                .expect("a valid latitude")
                .on_ellipsoid_m_s2())
        .abs()
            < 1e-15
    );
}

#[test]
fn normal_gravity_matches_the_exact_field_from_minus_5_to_20_km() {
    for (latitude_deg, height_m, exact) in EXACT {
        let model = normal_at(latitude_deg)
            .expect("a valid latitude")
            .at_m_s2(height_m);
        // The expansion in height is second order: 1.5e-6 m/s² is 1.5e-7 g.
        assert!(
            (model - exact).abs() < 1.5e-6,
            "{latitude_deg}°, {height_m} m: {model} against {exact}"
        );
        // And it is exact on the ellipsoid.
        if height_m == 0.0 {
            assert!(
                (model - exact).abs() < 1e-9,
                "{latitude_deg}° on the ellipsoid"
            );
        }
    }
}

#[test]
fn standard_gravity_is_the_same_everywhere_and_points_down() {
    for height_m in [-5000.0, 0.0, 20_000.0] {
        assert_eq!(
            Gravity::Standard.magnitude_m_s2(height_m).to_bits(),
            G0_M_S2.to_bits()
        );
    }
    let down = Gravity::Wgs84(normal_at(45.0).expect("a valid latitude")).acceleration_w(1000.0);
    assert_eq!((down.x, down.y), (0.0, 0.0));
    assert!((down.z - normal_at(45.0).expect("a valid latitude").at_m_s2(1000.0)).abs() < 1e-15);
    assert!((Gravity::Standard.acceleration_w(0.0).z - G0_M_S2).abs() < 1e-15);
}

#[test]
fn latitudes_beyond_the_poles_are_rejected() {
    let beyond = core::f64::consts::FRAC_PI_2 + 1e-9;
    for latitude_rad in [beyond, -beyond, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(
                NormalGravity::new(latitude_rad),
                Err(EnvironmentError::LatitudeOutOfRange { .. })
            ),
            "{latitude_rad}"
        );
    }
    assert!(
        (normal_at(90.0).expect("a valid latitude").latitude_rad() - core::f64::consts::FRAC_PI_2)
            .abs()
            < 1e-15
    );
    assert!(NormalGravity::new(-core::f64::consts::FRAC_PI_2).is_ok());
}

#[test]
fn the_environment_measures_altitude_from_the_field() {
    let environment = Environment::new(
        1500.0,
        mjk_env::Atmosphere::standard(),
        Gravity::Wgs84(normal_at(45.0).expect("a valid latitude")),
    )
    .expect("a finite elevation");
    // 200 m above the field: Down is −200 m.
    let position = Vec3::<World>::new(30.0, -40.0, -200.0);
    assert!((environment.altitude_m(position) - 1700.0).abs() < 1e-12);
    let air = environment.air(position).expect("in range");
    let direct = mjk_env::Atmosphere::standard()
        .at(1700.0)
        .expect("in range");
    assert_eq!(air, direct);
    assert!(
        (environment.gravity_w(position).z
            - normal_at(45.0).expect("a valid latitude").at_m_s2(1700.0))
        .abs()
            < 1e-15
    );
    assert!((environment.field_elevation_m() - 1500.0).abs() < 1e-15);
    assert_eq!(
        environment.gravity(),
        &Gravity::Wgs84(normal_at(45.0).expect("a valid latitude"))
    );
    assert_eq!(environment.atmosphere(), &mjk_env::Atmosphere::standard());
    // 25 km above sea level is above the atmosphere model.
    let high = Vec3::<World>::new(0.0, 0.0, -23_500.0);
    assert_eq!(
        environment.air(high),
        Err(EnvironmentError::AltitudeOutOfRange {
            altitude_m: 25_000.0
        })
    );
}

#[test]
fn a_field_elevation_must_be_finite() {
    for elevation in [f64::NAN, f64::INFINITY] {
        assert!(matches!(
            Environment::new(
                elevation,
                mjk_env::Atmosphere::standard(),
                Gravity::Standard
            ),
            Err(EnvironmentError::InvalidFieldElevation { .. })
        ));
    }
}

#[test]
fn errors_explain_themselves() {
    let messages = [
        (
            EnvironmentError::InvalidTemperatureOffset {
                temperature_offset_k: -300.0,
            },
            "a temperature offset of -300 K is not usable: it must be finite and leave the coldest layer above absolute zero",
        ),
        (
            EnvironmentError::AltitudeOutOfRange {
                altitude_m: 25_000.0,
            },
            "25000 m is outside the standard atmosphere's range, -5000 m to 20000 m",
        ),
        (
            EnvironmentError::LatitudeOutOfRange { latitude_rad: 2.0 },
            "a latitude of 2 rad is outside -pi/2 to pi/2",
        ),
        (
            EnvironmentError::InvalidFieldElevation {
                field_elevation_m: f64::INFINITY,
            },
            "a field elevation of inf m is not finite",
        ),
    ];
    for (error, message) in messages {
        assert_eq!(error.to_string(), message);
    }
}
