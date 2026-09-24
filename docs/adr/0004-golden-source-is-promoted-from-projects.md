# The golden source is promoted from projects

`reference/` is downstream of the first project that needs a thing and upstream of every project after it. A project may grow an implementation that is better than the golden source. Once that project is finished, the implementation is merged back into `reference/`, and later projects build on the golden source rather than writing their own.

The merge is an extraction, not a copy. The promoted code keeps the general core (for LQR: take `A, B, Q, R`, return the gain, the Riccati solution, the closed-loop poles and the residual) and leaves behind project-specific contracts such as error messages written for one UI. That keeps the golden source flexible enough for projects whose files will not look like the first one's.

## Consequences

- Before writing a numerical routine, an agent looks in `reference/` and `knowledge/`, and says what it found.
- Ball-balancer's own LQR, PID, servo lag and frame loop are candidates for promotion. Its LQR (matrix sign function with validation) is stronger than `reference/controllers/lqr.cpp` (Hamiltonian eigenvectors, no validation), and both declare `caliburn::LqrResult` differently. The merge resolves that collision.
