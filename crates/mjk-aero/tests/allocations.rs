//! An ESC update allocates nothing (#19): 10⁵ updates with a latency and a slew limit, under a
//! counting global allocator.
//!
//! This test runs without the test harness (`harness = false`), so no other thread allocates
//! while it measures; `main` panics through an assertion on any allocation.

use mjk_aero::propulsion::{Braking, Drive, Esc};
use mjk_alloc_counter::{CountingAllocator, allocations};

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const UPDATES: u32 = 100_000;

fn main() {
    let drive = Drive {
        braking: Braking::Active,
        current_limit_a: Some(20.0),
    };
    let esc = Esc::new(0.002, Some(40.0), drive, 1000);
    assert!(esc.is_ok(), "a usable ESC");
    let Ok(esc) = esc else { return };
    let mut state = esc.start(0.0);
    let before = allocations();
    for tick in 0..UPDATES {
        let command = if tick % 500 < 250 { 0.8 } else { 0.2 };
        let _: f64 = state.update(command);
    }
    let during = allocations() - before;
    assert_eq!(during, 0, "{UPDATES} ESC updates allocated {during} times");
    println!("allocations: {UPDATES} ESC updates with no allocation");
}
