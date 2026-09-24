# External numerical libraries are test-only oracles

SLICOT (the Fortran library behind MATLAB's `care`/`lqr`), SciPy, python-control/slycot, Drake and ethz-adrl/control-toolbox are used **only in tests**, as **oracles** that check the golden source's solvers. No project, and nothing in `reference/`, links them.

Linking them into projects was rejected for three reasons. Projects ship to the browser through Emscripten, and neither Drake nor control-toolbox cross-compiles to WebAssembly. The execution layer has to run on a microcontroller. And Caliburn exists to hold readable, cited implementations, which a call into a large framework does not provide.

## Consequences

- Tests that need an oracle live behind a CMake option and a ctest label. The default suite builds and passes without Drake, SLICOT or Python.
- Oracle results are also checked in as **oracle fixtures**, so the default suite still compares against them. The live oracle tests regenerate and cross-check those fixtures.
- Every solver in the golden source also checks itself where the maths allows: for a Riccati solution, the residual of the Riccati equation, computed without any oracle.
