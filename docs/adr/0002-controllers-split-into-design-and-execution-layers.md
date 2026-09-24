# Controllers split into a design layer and an execution layer

Every controller is two separate pieces. The **design layer** turns a tuning and a controller model into a **gain set**. It runs only on the desktop and may use Eigen with dynamic sizes, iterative solvers and error strings. The **execution layer** turns measurements and a gain set into actuator commands. It is isolated from the rest of the project: fixed-size, no heap allocation, no exceptions, no I/O, and a bounded worst-case run time. The same execution code runs in the simulation and on a microcontroller (ESP32, Raspberry Pi) wired to a physical system matching the model.

We rejected computing gains on the target. A Riccati solve needs an eigensolver, dynamic allocation and an unbounded number of iterations, none of which belong in a control interrupt, and a gain computed on the device cannot be tested against a stored expected result. Adapting to a changing system, when a project needs it, is done by gain scheduling: selecting among gain sets designed in advance.

## Consequences

- The design layer exports a gain set as a generated header that records where it came from: the controller model, the tuning, the commit, and the solver residual it was accepted at.
- A project may carry several controller strategies (LQR, LQI, sliding mode, …), each with its own saved tuning. A **controller switch** always resets the physical system first. There is no bumpless transfer, because the simulation has no need for mid-flight handover.
- The execution layer must not depend on the scene, the UI, or the physical system's code.
