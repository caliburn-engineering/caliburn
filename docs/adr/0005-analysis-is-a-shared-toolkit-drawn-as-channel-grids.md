# Analysis is a shared toolkit, drawn as channel grids

The analysis views (Bode, pole-zero, Nyquist, time response, and any added later) are a toolkit that each project pulls in piece by piece, not one analyzer application. Each piece takes only a state-space or transfer-function model together with its **signal names**, and depends on nothing project-specific. Asking for "a pole-zero map in my project" means the agent brings in that piece's golden source and knowledge file.

A multi-input, multi-output model is drawn as a **channel grid** (one plot per channel, rows for outputs, columns for inputs, labelled with signal names), not as a single plot with an input/output selector. The selector was rejected because it hides every channel except the selected one and labels channels by index.

## Consequences

- `LinearSystem` in the golden source carries signal names.
- In ball-balancer, `analysis_lib` currently links the table kinematics because the model library builds the cascade preset. That dependency has to be cut before the toolkit can be shared.
- The toolkit's maths is checked against oracles and against cases where the answer is known by hand (ADR-0003), before it is promoted.
