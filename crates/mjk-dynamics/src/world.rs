//! The fixed-step world loop (ADR 0002).
//!
//! Each physics step runs, in this order:
//!
//! 1. the model's discrete subsystems that are due on this tick, sensors before controllers,
//!    reading the state at the start of the step and updating inputs the dynamics then hold
//!    constant for the step (zero-order hold);
//! 2. one integrator step of the continuous dynamics: environment, actuators and forces are
//!    evaluated inside every stage of it;
//! 3. the model's after-step hook, where contact and logging attach.
//!
//! A step allocates nothing.

use crate::clock::{Clock, RateError};
use crate::integrator::{Integrator, System};

/// The coordinates of a model's continuous dynamics.
pub type ModelCoordinates<M> = <<M as Model>::System as System>::Coordinates;

/// What a world simulates: continuous dynamics, and the discrete subsystems that run on the
/// clock. A vehicle composes its sensors and controllers as fields and runs the due ones in a
/// fixed order in [`Model::update`].
pub trait Model {
    /// The continuous dynamics.
    type System: System;

    /// The continuous dynamics, with the inputs currently held.
    fn system(&self) -> &Self::System;

    /// Runs the subsystems due on the clock's tick, sensors before controllers, reading the state
    /// at the start of the step. Inputs they change are held for the whole step.
    fn update(&mut self, clock: &Clock, y: &ModelCoordinates<Self>);

    /// Runs after the state has advanced, with the clock already at the new tick: contact
    /// resolution and logging attach here. Does nothing by default.
    fn after_step(&mut self, _clock: &Clock, _y: &mut ModelCoordinates<Self>) {}
}

/// The outcome of [`World::advance`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Advance {
    /// Physics steps taken.
    pub steps: u32,
    /// How far real time has moved past the last step, as a fraction of a step in `[0, 1)`:
    /// draw `previous + alpha·(current − previous)`.
    pub alpha: f64,
    /// Real time discarded because the simulation could not keep up, in seconds.
    pub dropped_s: f64,
}

/// A model advanced in fixed steps by an integrator.
#[derive(Clone)]
pub struct World<M: Model, I: Integrator<M::System>> {
    model: M,
    integrator: I,
    clock: Clock,
    current: ModelCoordinates<M>,
    previous: ModelCoordinates<M>,
    accumulator_s: f64,
    max_steps_per_advance: u32,
}

impl<M: Model, I: Integrator<M::System>> World<M, I> {
    /// A world at tick zero, in state `y0`, stepping at `physics_hz`. [`World::advance`] takes
    /// at most a quarter of a second of steps per call.
    ///
    /// # Errors
    ///
    /// [`RateError::Zero`] for a physics rate of zero.
    pub fn new(
        model: M,
        integrator: I,
        y0: ModelCoordinates<M>,
        physics_hz: u32,
    ) -> Result<Self, RateError> {
        let clock = Clock::new(physics_hz)?;
        Ok(Self {
            model,
            integrator,
            clock,
            current: y0,
            previous: y0,
            accumulator_s: 0.0,
            max_steps_per_advance: (physics_hz / 4).max(1),
        })
    }

    /// The same world, taking at most `steps` steps per [`World::advance`] (at least one).
    #[must_use]
    pub fn with_max_steps_per_advance(mut self, steps: u32) -> Self {
        self.max_steps_per_advance = steps.max(1);
        self
    }

    /// Advances one physics step.
    pub fn step(&mut self) {
        self.model.update(&self.clock, &self.current);
        let next = self.integrator.step(
            self.model.system(),
            self.clock.time_s(),
            &self.current,
            self.clock.dt_s(),
        );
        self.previous = self.current;
        self.current = next;
        self.clock.advance();
        self.model.after_step(&self.clock, &mut self.current);
    }

    /// Advances by `real_dt_s` of real time for a real-time front end: as many whole steps as
    /// fit, carrying the remainder to the next call. If more steps are due than the cap allows,
    /// the whole backlog is dropped and the next call starts afresh, rather than letting the
    /// backlog grow (the "spiral of death"). A negative or non-finite interval counts as zero.
    pub fn advance(&mut self, real_dt_s: f64) -> Advance {
        if real_dt_s.is_finite() {
            self.accumulator_s += real_dt_s.max(0.0);
        }
        let dt_s = self.clock.dt_s();
        let mut steps = 0;
        for _ in 0..self.max_steps_per_advance {
            if self.accumulator_s < dt_s {
                break;
            }
            self.step();
            self.accumulator_s -= dt_s;
            steps += 1;
        }
        let mut dropped_s = 0.0;
        if self.accumulator_s >= dt_s {
            dropped_s = self.accumulator_s;
            self.accumulator_s = 0.0;
        }
        Advance {
            steps,
            alpha: self.accumulator_s / dt_s,
            dropped_s,
        }
    }

    /// The current state.
    #[must_use]
    pub const fn current(&self) -> &ModelCoordinates<M> {
        &self.current
    }

    /// The state one step earlier, for interpolating what is drawn.
    #[must_use]
    pub const fn previous(&self) -> &ModelCoordinates<M> {
        &self.previous
    }

    /// The clock.
    #[must_use]
    pub const fn clock(&self) -> &Clock {
        &self.clock
    }

    /// The model.
    #[must_use]
    pub const fn model(&self) -> &M {
        &self.model
    }

    /// The model, for changing inputs between steps, such as pilot commands.
    pub const fn model_mut(&mut self) -> &mut M {
        &mut self.model
    }
}
