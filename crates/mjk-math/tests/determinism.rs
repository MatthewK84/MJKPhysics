//! Seeded random numbers and state fingerprints (#14).
//!
//! Reference values come from a direct transcription of the published C code into Python
//! (`splitmix64.c` and `xoshiro256starstar.c` from prng.di.unimi.it, and FNV-1a 64). The first
//! `SplitMix64` output for seed 0, the xoshiro256** sequence from state [1, 2, 3, 4], and the
//! FNV-1a hashes of "a" and "foobar" also match values published independently.

use mjk_math::{
    Body, Mat3, Quat, Rotation, SplitMix64, StateHash, StateHasher, UnitQuat, Vec3, World,
    Xoshiro256StarStar, state_hash,
};

/// 2⁻⁵³.
const UNIT: f64 = 1.0 / 9_007_199_254_740_992.0;

fn draws(rng: &mut Xoshiro256StarStar, n: usize) -> Vec<u64> {
    (0..n).map(|_| rng.next_u64()).collect()
}

fn hash_bytes(bytes: &[u8]) -> u64 {
    let mut hasher = StateHasher::new();
    hasher.write_bytes(bytes);
    hasher.finish()
}

fn hash_f64s(values: &[f64]) -> u64 {
    let mut hasher = StateHasher::new();
    for value in values {
        hasher.write_u64(value.to_bits());
    }
    hasher.finish()
}

// ---------------------------------------------------------------- generators

#[test]
fn splitmix64_matches_the_reference() {
    let mut rng = SplitMix64::new(0);
    let values: Vec<u64> = (0..5).map(|_| rng.next_u64()).collect();
    assert_eq!(
        values,
        [
            16_294_208_416_658_607_535,
            7_960_286_522_194_355_700,
            487_617_019_471_545_679,
            17_909_611_376_780_542_444,
            1_961_750_202_426_094_747,
        ]
    );
}

#[test]
fn xoshiro256_star_star_matches_the_reference() {
    let mut rng = Xoshiro256StarStar::from_state([1, 2, 3, 4]).expect("nonzero");
    assert_eq!(
        draws(&mut rng, 10),
        [
            11_520,
            0,
            1_509_978_240,
            1_215_971_899_390_074_240,
            1_216_172_134_540_287_360,
            607_988_272_756_665_600,
            16_172_922_978_634_559_625,
            8_476_171_486_693_032_832,
            10_595_114_339_597_558_777,
            2_904_607_092_377_533_576,
        ]
    );
}

#[test]
fn a_seed_is_expanded_by_splitmix64() {
    let mut rng = Xoshiro256StarStar::from_seed(42);
    assert_eq!(
        rng.state(),
        [
            0xBDD7_3226_2FEB_6E95,
            0x28EF_E333_B266_F103,
            0x4752_6757_130F_9F52,
            0x581C_E1FF_0E4A_E394,
        ]
    );
    assert_eq!(
        draws(&mut rng, 5),
        [
            1_546_998_764_402_558_742,
            6_990_951_692_964_543_102,
            12_544_586_762_248_559_009,
            17_057_574_109_182_124_193,
            18_295_552_978_065_317_476,
        ]
    );
}

#[test]
fn only_the_all_zero_state_is_rejected() {
    assert_eq!(Xoshiro256StarStar::from_state([0; 4]), None);
    for i in 0..4 {
        let mut s = [0; 4];
        s[i] = 1;
        assert!(Xoshiro256StarStar::from_state(s).is_some(), "{s:?}");
    }
    // Words that cancel in pairs are still a nonzero state.
    for s in [[1, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1], [u64::MAX; 4]] {
        assert!(Xoshiro256StarStar::from_state(s).is_some(), "{s:?}");
    }
}

// ---------------------------------------------------------------- named streams

#[test]
fn a_named_stream_is_seeded_from_the_master_seed_and_its_name() {
    let mut gyro = Xoshiro256StarStar::named(42, "gyro");
    assert_eq!(
        gyro.state(),
        [
            0xBDD7_3226_2FEB_6E95,
            0x8CEE_F3EC_DF02_E21D,
            0xF6C4_97A2_D244_EE2C,
            0xF99D_5D2A_FBDD_E68B,
        ]
    );
    assert_eq!(
        draws(&mut gyro, 3),
        [
            31_614_918_199_316_448,
            14_130_086_764_394_278_411,
            9_070_051_639_082_525_656,
        ]
    );
}

#[test]
fn a_stream_state_is_the_mixed_seed_then_the_name_chained_to_it() {
    let state = Xoshiro256StarStar::named(42, "gyro").state();
    let mixed_seed = SplitMix64::new(42).next_u64();
    let mut chained = SplitMix64::new(hash_bytes(b"gyro") ^ mixed_seed);
    let expected = [
        mixed_seed,
        chained.next_u64(),
        chained.next_u64(),
        chained.next_u64(),
    ];
    assert_eq!(state, expected);
}

#[test]
fn no_two_seed_and_name_pairs_share_a_stream() {
    // XORing the seed with the name's hash, as this generator first did, made seed 7's gyro
    // stream and this seed's pilot stream identical.
    let partner = 7 ^ hash_bytes(b"gyro") ^ hash_bytes(b"pilot");
    assert_eq!(partner, 8_938_727_255_507_050_944);
    let xor_seeded =
        |seed: u64, name: &[u8]| Xoshiro256StarStar::from_seed(seed ^ hash_bytes(name));
    assert_eq!(xor_seeded(7, b"gyro"), xor_seeded(partner, b"pilot"));
    assert_ne!(
        Xoshiro256StarStar::named(7, "gyro").state(),
        Xoshiro256StarStar::named(partner, "pilot").state()
    );
}

#[test]
fn streams_start_with_different_draws() {
    // The first draw depends only on the second word of the state, so that word must depend on
    // both the seed and the name.
    let in_one_run: Vec<u64> = ["gyro", "accelerometer", "pilot"]
        .iter()
        .map(|name| Xoshiro256StarStar::named(42, name).next_u64())
        .collect();
    let across_runs: Vec<u64> = [1, 2, 3]
        .iter()
        .map(|seed| Xoshiro256StarStar::named(*seed, "gyro").next_u64())
        .collect();
    for firsts in [in_one_run, across_runs] {
        let mut distinct = firsts.clone();
        distinct.sort_unstable();
        distinct.dedup();
        assert_eq!(distinct.len(), firsts.len(), "{firsts:?}");
    }
}

#[test]
fn adding_a_stream_does_not_change_another() {
    let alone = draws(&mut Xoshiro256StarStar::named(7, "turbulence"), 100);
    let mut sensor = Xoshiro256StarStar::named(7, "accelerometer");
    let mut turbulence = Xoshiro256StarStar::named(7, "turbulence");
    let mut interleaved = Vec::new();
    for _ in 0..100 {
        sensor.next_u64();
        interleaved.push(turbulence.next_u64());
        sensor.next_u64();
    }
    assert_eq!(interleaved, alone);
}

#[test]
fn streams_differ_by_name_and_by_seed() {
    let base = draws(&mut Xoshiro256StarStar::named(7, "gyro"), 4);
    assert_ne!(draws(&mut Xoshiro256StarStar::named(7, "gyro2"), 4), base);
    assert_ne!(draws(&mut Xoshiro256StarStar::named(8, "gyro"), 4), base);
}

// ---------------------------------------------------------------- distributions

#[test]
fn a_uniform_sample_is_the_top_53_bits_scaled_into_the_unit_interval() {
    // From [1, 2, 3, 4] the first output is 11 520 = 5·2¹¹, so the top 53 bits are 5.
    let mut rng = Xoshiro256StarStar::from_state([1, 2, 3, 4]).expect("nonzero");
    assert_eq!(rng.next_f64().to_bits(), (5.0 * UNIT).to_bits());
    // The second output is zero.
    assert_eq!(rng.next_f64().to_bits(), 0.0_f64.to_bits());
    // A state whose next output is all ones gives the largest value, just below 1.
    let mut top =
        Xoshiro256StarStar::from_state([0, 0x4FC7_1C71_C71C_71C7, 0, 0]).expect("nonzero");
    assert_eq!(top.next_f64().to_bits(), (1.0 - UNIT).to_bits());
}

#[test]
fn uniform_samples_have_the_right_mean_and_spread() {
    let mut rng = Xoshiro256StarStar::named(1, "uniform");
    let samples: Vec<f64> = (0..100_000).map(|_| rng.next_f64()).collect();
    assert!(samples.iter().all(|u| (0.0..1.0).contains(u)));
    let mean = samples.iter().sum::<f64>() / 100_000.0;
    let variance = samples.iter().map(|u| (u - 0.5) * (u - 0.5)).sum::<f64>() / 100_000.0;
    assert!((mean - 0.5).abs() < 0.005, "{mean}");
    assert!((variance - 1.0 / 12.0).abs() < 0.002, "{variance}");
}

#[test]
fn gaussian_samples_match_the_box_muller_reference() {
    let mut rng = Xoshiro256StarStar::named(7, "turbulence");
    let expected = [
        -0.919_986_653_515_486_3,
        2.909_483_802_984_310_4,
        -0.087_976_056_573_552_15,
    ];
    for value in expected {
        let sample = rng.next_gaussian();
        assert!((sample - value).abs() < 1e-14, "{sample} against {value}");
    }
}

#[test]
fn gaussian_samples_have_unit_variance_and_normal_tails() {
    let mut rng = Xoshiro256StarStar::named(3, "noise");
    let samples: Vec<f64> = (0..100_000).map(|_| rng.next_gaussian()).collect();
    let mean = samples.iter().sum::<f64>() / 100_000.0;
    let variance = samples.iter().map(|z| z * z).sum::<f64>() / 100_000.0;
    let within_one = samples.iter().filter(|z| z.abs() < 1.0).count();
    assert!(mean.abs() < 0.02, "{mean}");
    assert!((variance - 1.0).abs() < 0.03, "{variance}");
    // 68.27 % of a normal distribution lies within one standard deviation.
    assert!((67_800..68_700).contains(&within_one), "{within_one}");
}

// ---------------------------------------------------------------- fingerprints

#[test]
fn the_hasher_is_fnv_1a_64() {
    assert_eq!(hash_bytes(b""), 0xCBF2_9CE4_8422_2325);
    assert_eq!(hash_bytes(b"a"), 0xAF63_DC4C_8601_EC8C);
    assert_eq!(hash_bytes(b"foobar"), 0x8594_4171_F739_67E8);
    assert_eq!(StateHasher::default(), StateHasher::new());
}

#[test]
fn integers_go_in_as_little_endian_bytes() {
    let mut hasher = StateHasher::new();
    hasher.write_u64(0x0102_0304_0506_0708);
    assert_eq!(hasher.finish(), hash_bytes(&[8, 7, 6, 5, 4, 3, 2, 1]));
    assert_eq!(state_hash(&7_u32), state_hash(&7_u64));
}

#[test]
fn floats_go_in_as_their_raw_bits() {
    assert_eq!(state_hash(&1.5), hash_f64s(&[1.5]));
    assert_ne!(state_hash(&0.0), state_hash(&-0.0));
    assert_ne!(state_hash(&[1.0, 2.0]), state_hash(&[2.0, 1.0]));
}

#[test]
fn math_types_hash_their_components_in_order() {
    let v = Vec3::<Body>::new(1.0, -2.0, 3.5);
    assert_eq!(state_hash(&v), hash_f64s(&[1.0, -2.0, 3.5]));
    let m = Mat3::<Body>::from_rows([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
    let entries: Vec<f64> = (1..=9).map(f64::from).collect();
    assert_eq!(state_hash(&m), hash_f64s(&entries));
    let q = Quat::new(0.5, 0.5, -0.5, 0.5);
    assert_eq!(state_hash(&q), hash_f64s(&[0.5, 0.5, -0.5, 0.5]));
    let unit = UnitQuat::new(q).expect("unit");
    assert_eq!(state_hash(&unit), state_hash(&q));
    let rotation = Rotation::<World, Body>::from_unit_quat(unit);
    assert_eq!(state_hash(&rotation), state_hash(&q));
}

#[test]
fn a_generator_hashes_its_state() {
    let mut rng = Xoshiro256StarStar::from_state([1, 2, 3, 4]).expect("nonzero");
    let mut expected = StateHasher::new();
    for word in [1_u64, 2, 3, 4] {
        word.hash_state(&mut expected);
    }
    assert_eq!(state_hash(&rng), expected.finish());
    rng.next_u64();
    assert_ne!(state_hash(&rng), expected.finish());
}
