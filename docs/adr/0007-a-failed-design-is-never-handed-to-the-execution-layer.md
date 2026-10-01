# A failed design is never handed to the execution layer

The design layer can fail. A Riccati solve whose relative residual exceeds the gate returns no gain rather than a wrong one (ADR-0003), and a physical parameter can move the plant into a region where no stabilizing design exists at all. When the design fails, the execution layer must receive **nothing** — not the last good gain, not a partial one, not a NaN wearing the shape of a gain.

Concretely, on a failed design: no gain reaches the loop; the loop does not engage, or drops if it was engaged; and the simulation holds a safe pose rather than freezing at a stale command. A single mathematical failure must never propagate into the model → controller → simulation → visualization pipeline as a stale actuation or a real-time crash.

This is the safety rule on the design/execution seam that ADR-0002 already draws across every project. It is stated here, at the workspace level, because it is not specific to any one project: any project that designs a controller and then executes it inherits this seam, and inherits this rule on it.

## Consequences

- The predicate that decides whether a design is **usable** — offered, well-formed, and for the plant actually being simulated — is a pure, testable function, not logic buried in a render loop where no test can reach it.
- A failed solve clears any stored gain, so a later step cannot pick up a gain designed against a plant that no longer exists.
- The drop distinguishes a transient numerical failure from a deliberate stop: a loop dropped because the solve failed re-engages when the design becomes usable again, while a loop the operator stopped stays stopped.
- The invariant is asserted end to end by a test that drives a failed design through the seam and confirms the executed system stays at its safe pose.
