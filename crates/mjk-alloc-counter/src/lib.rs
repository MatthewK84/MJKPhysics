//! A counting global allocator, for tests that prove code does not allocate.
//!
//! This crate holds the only `unsafe` code in the repository: implementing [`GlobalAlloc`] is
//! unsafe by definition. Every method forwards to the system allocator unchanged and counts the
//! call in an atomic integer. It is a development dependency only and is never part of a build
//! that ships.
//!
//! A test installs it with `#[global_allocator]` and compares [`allocations`] before and after the
//! code under test. Run such tests in their own test binary without the test harness, so no other
//! thread allocates while they measure.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicU64, Ordering};

static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);

/// Counts every allocation and reallocation, then delegates to the system allocator.
#[derive(Clone, Copy, Debug, Default)]
pub struct CountingAllocator;

#[allow(unsafe_code, reason = "GlobalAlloc can only be implemented unsafely")]
// SAFETY: every method passes its arguments unchanged to `System`, which upholds the
// `GlobalAlloc` contract; the counting only touches an atomic integer and never allocates.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        // SAFETY: the caller's guarantees about `layout` are passed on unchanged.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        // SAFETY: the caller's guarantees about `layout` are passed on unchanged.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        // SAFETY: `ptr` came from this allocator, so from `System`, with this `layout`.
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: `ptr` came from this allocator, so from `System`, with this `layout`.
        unsafe { System.dealloc(ptr, layout) }
    }
}

/// Allocations and reallocations so far, in every thread.
#[must_use]
pub fn allocations() -> u64 {
    ALLOCATIONS.load(Ordering::SeqCst)
}
