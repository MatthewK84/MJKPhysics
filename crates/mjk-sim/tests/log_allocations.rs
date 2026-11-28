//! Logging a row allocates nothing (#17): 10⁵ rows into a log sized for them, under a counting
//! global allocator.
//!
//! This test runs without the test harness (`harness = false`), so no other thread allocates
//! while it measures; `main` panics through an assertion on any allocation.

use mjk_alloc_counter::{CountingAllocator, allocations};
use mjk_sim::log::{Channel, ColumnLog};

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

const ROWS: usize = 100_000;

const CHANNELS: [Channel; 3] = [
    Channel {
        name: "time",
        unit: "s",
    },
    Channel {
        name: "north",
        unit: "m",
    },
    Channel {
        name: "down",
        unit: "m",
    },
];

fn main() {
    let mut log = ColumnLog::new(&CHANNELS, ROWS);
    let before = allocations();
    for row in 0..ROWS {
        let t = f64::from(u32::try_from(row).unwrap_or(u32::MAX)) * 1e-3;
        assert!(log.push(&[t, 2.0 * t, -t]).is_ok(), "sized for every row");
    }
    let during = allocations() - before;
    assert_eq!(during, 0, "logging {ROWS} rows allocated {during} times");
    assert_eq!(log.rows(), ROWS);
    println!("log_allocations: {ROWS} rows logged with no allocation");
}
