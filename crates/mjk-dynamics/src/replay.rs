//! Checkpoint hashes and input replay (#14).
//!
//! A run is reproducible when the same seed and the same inputs, applied at the same ticks, give
//! the same state to the bit. [`Checkpoints`] records the state's fingerprint at a fixed rate, so
//! two runs can be compared tick by tick. [`InputLog`] records each input with the tick it was
//! applied at, and [`Replay`] hands the inputs back at the same ticks.
//!
//! Both records are sized when they are made and never grow, so recording allocates nothing
//! inside a step. A full record is an error, never a silent drop.

use std::fmt;

use crate::clock::Rate;
use crate::world::Model;

/// Why an entry could not be recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RecordError {
    /// The record already holds as many entries as it was sized for.
    Full {
        /// The number of entries it holds.
        capacity: usize,
    },
    /// The entry's tick comes before the last one recorded.
    OutOfOrder {
        /// The entry's tick.
        tick: u64,
        /// The tick of the last entry recorded.
        last_tick: u64,
    },
}

impl fmt::Display for RecordError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Full { capacity } => write!(
                f,
                "the record is full at {capacity} entries; size it for the whole run"
            ),
            Self::OutOfOrder { tick, last_tick } => write!(
                f,
                "tick {tick} comes before tick {last_tick}, the last one recorded"
            ),
        }
    }
}

/// A state fingerprint at one tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Checkpoint {
    /// The tick the state was hashed at.
    pub tick: u64,
    /// The state's fingerprint.
    pub hash: u64,
}

/// State fingerprints taken at a fixed rate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Checkpoints {
    rate: Rate,
    capacity: usize,
    records: Vec<Checkpoint>,
}

impl Checkpoints {
    /// An empty record taking a checkpoint at `rate`, with room for `capacity` of them.
    #[must_use]
    pub fn new(rate: Rate, capacity: usize) -> Self {
        Self {
            rate,
            capacity,
            records: Vec::with_capacity(capacity),
        }
    }

    /// True on the ticks a checkpoint is due: zero, then every rate divisor.
    #[must_use]
    pub fn is_due(&self, tick: u64) -> bool {
        self.rate.is_due(tick)
    }

    /// Records the fingerprint `hash` at `tick`.
    ///
    /// # Errors
    ///
    /// [`RecordError::Full`] when the record is full, and [`RecordError::OutOfOrder`] unless
    /// `tick` is later than the last checkpoint.
    pub fn record(&mut self, tick: u64, hash: u64) -> Result<(), RecordError> {
        if let Some(last) = self.records.last()
            && tick <= last.tick
        {
            return Err(RecordError::OutOfOrder {
                tick,
                last_tick: last.tick,
            });
        }
        if self.records.len() >= self.capacity {
            return Err(RecordError::Full {
                capacity: self.capacity,
            });
        }
        self.records.push(Checkpoint { tick, hash });
        Ok(())
    }

    /// The checkpoints so far, in tick order.
    #[must_use]
    pub fn records(&self) -> &[Checkpoint] {
        &self.records
    }
}

/// A model that takes inputs between steps, such as pilot commands.
pub trait InputModel: Model {
    /// One input.
    type Input: Copy;

    /// Applies `input`. It is held, like a controller's output, from the next step on.
    fn apply(&mut self, input: &Self::Input);
}

/// One input and the tick it was applied at, before that tick's step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputEvent<I> {
    /// The tick.
    pub tick: u64,
    /// The input.
    pub input: I,
}

/// The inputs applied during a run, in order.
#[derive(Clone, Debug, PartialEq)]
pub struct InputLog<I> {
    capacity: usize,
    events: Vec<InputEvent<I>>,
}

impl<I: Copy> InputLog<I> {
    /// An empty log with room for `capacity` inputs.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            capacity,
            events: Vec::with_capacity(capacity),
        }
    }

    /// Records `input` as applied at `tick`. Several inputs may share a tick.
    ///
    /// # Errors
    ///
    /// [`RecordError::Full`] when the log is full, and [`RecordError::OutOfOrder`] when `tick`
    /// is earlier than the last input's.
    pub fn record(&mut self, tick: u64, input: I) -> Result<(), RecordError> {
        if let Some(last) = self.events.last()
            && tick < last.tick
        {
            return Err(RecordError::OutOfOrder {
                tick,
                last_tick: last.tick,
            });
        }
        if self.events.len() >= self.capacity {
            return Err(RecordError::Full {
                capacity: self.capacity,
            });
        }
        self.events.push(InputEvent { tick, input });
        Ok(())
    }

    /// The inputs, in the order they were applied.
    #[must_use]
    pub fn events(&self) -> &[InputEvent<I>] {
        &self.events
    }
}

/// Plays back an [`InputLog`], handing each input back on its tick.
#[derive(Clone, Copy, Debug)]
pub struct Replay<'log, I> {
    events: &'log [InputEvent<I>],
    next: usize,
}

impl<'log, I: Copy> Replay<'log, I> {
    /// A replay of `log` from its first input.
    #[must_use]
    pub fn new(log: &'log InputLog<I>) -> Self {
        Self {
            events: log.events(),
            next: 0,
        }
    }

    /// The inputs not yet handed back whose tick is at or before `tick`, in recorded order.
    /// Called before every step, it returns exactly the inputs applied before that step.
    pub fn due(&mut self, tick: u64) -> &'log [InputEvent<I>] {
        let rest = self.events.get(self.next..).unwrap_or_default();
        let count = rest.iter().take_while(|event| event.tick <= tick).count();
        self.next += count;
        rest.get(..count).unwrap_or_default()
    }

    /// True once every input has been handed back.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        self.next >= self.events.len()
    }
}
