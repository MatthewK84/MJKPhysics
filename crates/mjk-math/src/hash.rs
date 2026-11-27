//! State fingerprints for determinism checks (#14).
//!
//! A [`StateHasher`] folds the raw bits of a state into a 64-bit FNV-1a hash. Two states hash
//! equal only if every `f64` in them has the same bits, so `0.0` and `-0.0` differ, as they
//! should when the claim is bit-for-bit reproducibility. Bytes go in little-endian on every
//! platform, so a hash means the same thing natively and in WebAssembly.
//!
//! FNV-1a is a fingerprint, not a cryptographic hash. Each step is a bijection of the hash
//! state, so changing any single byte of the input always changes the result.

use crate::frame::Frame;
use crate::mat3::Mat3;
use crate::quat::{Quat, UnitQuat};
use crate::rotation::Rotation;
use crate::vec3::Vec3;

/// The FNV-1a 64-bit offset basis.
const OFFSET_BASIS: u64 = 0xCBF2_9CE4_8422_2325;

/// The FNV-1a 64-bit prime.
const PRIME: u64 = 0x0000_0100_0000_01B3;

/// An FNV-1a 64-bit hash of a sequence of bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StateHasher {
    hash: u64,
}

impl Default for StateHasher {
    fn default() -> Self {
        Self::new()
    }
}

impl StateHasher {
    /// A hasher that has seen nothing.
    #[must_use]
    pub const fn new() -> Self {
        Self { hash: OFFSET_BASIS }
    }

    /// Adds bytes in order, Eq. (DET-5).
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.hash = (self.hash ^ u64::from(*byte)).wrapping_mul(PRIME);
        }
    }

    /// Adds a `u64` as eight little-endian bytes.
    pub fn write_u64(&mut self, value: u64) {
        self.write_bytes(&value.to_le_bytes());
    }

    /// Adds the raw bits of an `f64`.
    pub fn write_f64(&mut self, value: f64) {
        self.write_u64(value.to_bits());
    }

    /// The hash of everything added so far.
    #[must_use]
    pub const fn finish(&self) -> u64 {
        self.hash
    }
}

/// A value whose exact state can be fingerprinted.
///
/// An implementation adds every field that can affect what happens next, in a fixed order.
pub trait StateHash {
    /// Adds this value's state to `hasher`.
    fn hash_state(&self, hasher: &mut StateHasher);
}

/// The fingerprint of one value.
#[must_use]
pub fn state_hash<T: StateHash + ?Sized>(value: &T) -> u64 {
    let mut hasher = StateHasher::new();
    value.hash_state(&mut hasher);
    hasher.finish()
}

impl StateHash for u64 {
    fn hash_state(&self, hasher: &mut StateHasher) {
        hasher.write_u64(*self);
    }
}

impl StateHash for u32 {
    fn hash_state(&self, hasher: &mut StateHasher) {
        hasher.write_u64(u64::from(*self));
    }
}

impl StateHash for f64 {
    fn hash_state(&self, hasher: &mut StateHasher) {
        hasher.write_f64(*self);
    }
}

impl<T: StateHash> StateHash for [T] {
    fn hash_state(&self, hasher: &mut StateHasher) {
        for value in self {
            value.hash_state(hasher);
        }
    }
}

impl<T: StateHash, const N: usize> StateHash for [T; N] {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.as_slice().hash_state(hasher);
    }
}

impl<F: Frame> StateHash for Vec3<F> {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.to_array().hash_state(hasher);
    }
}

impl<F: Frame> StateHash for Mat3<F> {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.rows().hash_state(hasher);
    }
}

impl StateHash for Quat {
    fn hash_state(&self, hasher: &mut StateHasher) {
        [self.w, self.x, self.y, self.z].hash_state(hasher);
    }
}

impl StateHash for UnitQuat {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.quat().hash_state(hasher);
    }
}

impl<To: Frame, From: Frame> StateHash for Rotation<To, From> {
    fn hash_state(&self, hasher: &mut StateHasher) {
        self.unit_quat().hash_state(hasher);
    }
}
