# The physical model and the controller model are separate

Every project keeps three things apart: the **physical system** (the simulated system, nonlinear where the real one is), the **physical model** (its linearization, which follows every physical parameter), and the **controller model** (a snapshot of the physical model that the controller is designed against). By default the two models are equal. When a physical parameter changes, the physical system, the physical model and the scene all change with it, but the controller model and its gain set stay as they were.

We chose this over a single model, which the first project used, so that the user can see what model mismatch costs: they change the plate radius, see the controller perform worse, and compare the physical model's Bode and pole-zero plots with the controller model's. It also covers wear and tear, and comparing how controller strategies cope with a model that is wrong.

## Consequences

- The UI shows model mismatch whenever the two models differ, and offers a **resync**, which copies the physical model into the controller model and redesigns the gain set.
- Every analysis view can be drawn for either model, and for both at once.
- A physical parameter has to reach the scene as well as the linearization. In ball-balancer today the scene is built from the mechanism once and is not rebuilt when a parameter changes.
- The physical model should be derived from the physical system (numerically linearized) or checked against it. A hand-derived model that nothing compares with the simulation can drift away from it without anyone noticing.
