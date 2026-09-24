# Caliburn

An agent-first control-theory builder: the workspace in which an agent helps a
user model a system, design a controller for it, and simulate both. These terms
hold across every project; a project's own `CONTEXT.md` adds its domain on top.

## Workspace

**Golden source**:
The tested, standalone implementations in `reference/` that every project draws on, grown from the projects that first needed them.
_Avoid_: library, vendor code, shared code

**Knowledge file**:
A cited note in `knowledge/` explaining one piece of theory, linked to the golden source that implements it.
_Avoid_: doc, article

**Project**:
One user system built in `projects/`, in its own git repository, from model through controller to simulation.
_Avoid_: app, example

## Models

**Physical system**:
The simulated system itself — nonlinear where the real one is — which actuators act on, sensors measure, and disturbances and noise enter.
_Avoid_: plant, real system, true plant

**Physical parameter**:
A user-editable property of the physical system, such as a plate radius or a mass.
_Avoid_: model parameter, constant

**Physical model**:
The linear model of the physical system at its operating point, which follows every change to a physical parameter.
_Avoid_: plant model, linearization

**Controller model**:
The linear model a controller is designed against: a snapshot of the physical model that stays put when physical parameters change.
_Avoid_: nominal model, design model, plant

**Model mismatch**:
The state in which the physical model and the controller model differ.
_Avoid_: drift, detuning

**Resync**:
Replacing the controller model with the current physical model, which redesigns the controller's gain set.
_Avoid_: update, refresh

## Controllers

**Tuning**:
The design inputs a controller strategy takes — weights, pole locations, gains to sweep — kept per strategy.
_Avoid_: settings, parameters

**Gain set**:
The matrices a design produces from a tuning and a controller model, and the only thing the execution layer receives.
_Avoid_: controller matrices, tuning, controller

**Design layer**:
The part of a controller that turns a tuning into a gain set; it runs on the desktop only.
_Avoid_: solver, offline part

**Execution layer**:
The part of a controller that turns measurements and a gain set into actuator commands, identical in simulation and on a microcontroller.
_Avoid_: runtime, controller loop, firmware

**Controller switch**:
Changing the active controller strategy, always preceded by a reset of the physical system.
_Avoid_: handover, bumpless transfer

## Simulation

**Frame**:
One simulation timestep, defined by exactly one function that the application and every test call.
_Avoid_: tick, step, update

**Scene**:
The OpenGL rendering of the physical system, drawn from its current physical parameters.
_Avoid_: view, graphics, render

## Analysis

**Signal names**:
The names a model carries for each of its states, inputs and outputs, shown wherever a signal is.
_Avoid_: labels, indices

**Channel**:
One input-to-output path through a model.
_Avoid_: pair, element, entry

**Channel grid**:
An analysis view drawn once per channel, arranged with one row per output and one column per input.
_Avoid_: pairing grid, matrix plot, subplot grid

## Verification

**Oracle**:
A trusted external numerical solver — SLICOT, SciPy, Drake, control-toolbox — used only by tests to check the golden source.
_Avoid_: reference solver, ground truth

**Oracle fixture**:
An oracle's results, checked into the repository as data so tests can compare against them without the oracle installed.
_Avoid_: golden data, snapshot
