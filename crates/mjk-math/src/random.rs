//! Seeded pseudorandom numbers (#14).
//!
//! [`Xoshiro256StarStar`] is Blackman and Vigna's xoshiro256** generator, written out from their
//! reference C code. It uses only integer arithmetic, so a seed gives the same sequence on every
//! platform. Seeds are expanded with [`SplitMix64`], as its authors recommend.
//!
//! Each subsystem draws from its own named stream, seeded from the run's master seed and the
//! stream's name. A stream's sequence depends on nothing else, so adding a sensor cannot change
//! the turbulence a run sees.

use crate::hash::{StateHash, StateHasher};

/// 2⁻⁵³, the spacing of the `f64` values [`Xoshiro256StarStar::next_f64`] returns.
const UNIT: f64 = 1.0 / 9_007_199_254_740_992.0;

/// 2³², as an `f64`.
const TWO_POW_32: f64 = 4_294_967_296.0;

/// Steele, Lea and Flood's `SplitMix64`, used to expand one 64-bit seed into a generator state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    /// A generator whose first output depends only on `seed`.
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// The next 64 random bits.
    pub const fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

/// The xoshiro256** generator: 256 bits of state, period 2²⁵⁶ − 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Xoshiro256StarStar {
    s: [u64; 4],
}

impl Xoshiro256StarStar {
    /// A generator from its raw state, or `None` for the all-zero state, which never leaves zero.
    #[must_use]
    pub const fn from_state(s: [u64; 4]) -> Option<Self> {
        if s[0] | s[1] | s[2] | s[3] == 0 {
            return None;
        }
        Some(Self { s })
    }

    /// A generator seeded from one `u64`, through four outputs of [`SplitMix64`]. The four
    /// outputs come from four different `SplitMix64` states and its output step is a bijection,
    /// so at most one is zero and the state is never all zero.
    #[must_use]
    pub const fn from_seed(seed: u64) -> Self {
        let mut expand = SplitMix64::new(seed);
        let s = [
            expand.next_u64(),
            expand.next_u64(),
            expand.next_u64(),
            expand.next_u64(),
        ];
        Self { s }
    }

    /// The stream called `name` in a run seeded with `master_seed`, Eq. (DET-2). It is the same
    /// however many other streams exist and in whatever order they were made.
    ///
    /// The master seed is mixed by one step of [`SplitMix64`]. A second `SplitMix64`, seeded with
    /// the FNV-1a hash of the name XOR that mixed seed, gives four outputs: the first three are
    /// words 1 to 3 of the state, and the fourth, XOR the mixed seed, is word 0. So no word is
    /// shared by the streams of one run.
    ///
    /// - Two streams start from the same state only if their seeds and their name hashes are
    ///   both equal: word 1 gives back the second generator's seed, which gives the fourth
    ///   output, which with word 0 gives back the mixed seed and so the master seed.
    /// - Word 1, which alone decides the first draw, depends on both the seed and the name.
    /// - Words 1 and 2 come from two different `SplitMix64` states, so at most one is zero and
    ///   the state is never all zero.
    #[must_use]
    pub fn named(master_seed: u64, name: &str) -> Self {
        let mut hasher = StateHasher::new();
        hasher.write_bytes(name.as_bytes());
        let mixed_seed = SplitMix64::new(master_seed).next_u64();
        let mut chained = SplitMix64::new(hasher.finish() ^ mixed_seed);
        let s1 = chained.next_u64();
        let s2 = chained.next_u64();
        let s3 = chained.next_u64();
        Self {
            s: [mixed_seed ^ chained.next_u64(), s1, s2, s3],
        }
    }

    /// The raw state.
    #[must_use]
    pub const fn state(&self) -> [u64; 4] {
        self.s
    }

    /// The next 64 random bits, Eq. (DET-1).
    pub const fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// A uniform `f64` in `[0, 1)`: the top 53 bits of [`Self::next_u64`], times 2⁻⁵³, Eq.
    /// (DET-3). Every value is a multiple of 2⁻⁵³ and each is equally likely.
    pub fn next_f64(&mut self) -> f64 {
        let bits = self.next_u64() >> 11;
        let high = u32::try_from(bits >> 32).unwrap_or(u32::MAX);
        let low = u32::try_from(bits & 0xFFFF_FFFF).unwrap_or(u32::MAX);
        // Exact: `bits` is below 2⁵³, and scaling by a power of two only changes the exponent.
        (f64::from(high) * TWO_POW_32 + f64::from(low)) * UNIT
    }

    /// A standard normal sample (mean 0, variance 1), by the Box–Muller transform, Eq. (DET-4).
    /// Each sample draws two uniforms and the second normal of the pair is not kept, so the state
    /// is the generator alone. The logarithm and cosine come from `libm`, so the sample is
    /// identical on every platform.
    pub fn next_gaussian(&mut self) -> f64 {
        // In (0, 1], so the logarithm is finite.
        let u1 = 1.0 - self.next_f64();
        let u2 = self.next_f64();
        libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(core::f64::consts::TAU * u2)
    }
}

impl StateHash for Xoshiro256StarStar {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.s.hash_state(hasher);
    }
}
