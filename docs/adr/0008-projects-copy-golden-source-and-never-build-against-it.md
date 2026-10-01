# Projects copy golden source and never build against it

The golden source in `reference/` is for reading and copying. No build file in a project may point at `reference/`, or at anything else outside the project's own repository. When a project needs golden source, it copies what it needs into its own code: a whole file goes straight into `src/`, and part of a file goes into whichever project source file it fits. From then on the copy is the project's own code, which it may edit freely. The person doing the work decides how much to copy.

A project is its own git repository (CONTEXT.md), and anyone should be able to clone one by itself, build it and run it. Ball-balancer broke that rule by compiling `rk4.cpp`, `linearizer.cpp` and `rolling_dynamics.cpp` out of `../../reference`. A lone clone failed to configure, which shelved its CI (#40), and AFK runs needed a bind-mount of `reference/` to stand in for the missing tree (#58). The tools Caliburn uses (GitHub, Docker, Sandcastle and the agent harnesses) are a convenient way to work on a project, not a requirement for building it.

## Considered Options

- **Fetch `reference/` as a pinned dependency** (CMake `FetchContent` of the caliburn repo, or a git submodule). This made a lone clone build, and it matched how Eigen and ImGui arrive. It was rejected because the project would still build against Caliburn rather than standing on its own, and `reference/` would have to grow library targets and guard its test build to serve as a dependency.
- **Install `reference/` as a prebuilt library** that projects find with `find_package`. This was rejected because every consumer compiles from source. The browser build is compiled by Emscripten, the desktop tests are compiled locally, and the execution layer is cross-compiled for the microcontroller (ADR-0002). No consumer, present or planned, can only take a binary.

## Consequences

- Every copy carries an origin comment naming the golden file and the caliburn commit it was copied at, for example `// From caliburn reference/integrators/rk4.cpp @ 49e8551`. For an excerpt, the comment goes directly above the pasted part. The commit lets ADR-0004's promotion tell a deliberate improvement apart from a copy of an older golden version. There is no manifest, no sync tool and no staleness check, because copies never follow later changes to `reference/`.
- Golden tests stay in `reference/`, where they check the golden source. A project tests what matters to it, and that often includes code it copied.
- "Standalone" means a machine with git, CMake and a C++ compiler can clone the project, configure it, build it and pass `ctest`. Downloading public third-party libraries during configure is allowed. The browser build is not part of this promise.
- `reference/CMakeLists.txt` stays a test-only build. It needs no library targets, install rules or package config.
- Ball-balancer copies the seven golden files it uses (`rk4`, `linearizer`, `linear_system`, `rolling_dynamics`) whole and unchanged into `src/`, apart from the origin comment. Its existing tests show that nothing changed, and its standalone CI (#40) is the proof that it builds on its own. Ball-balancer-legacy also compiles from `reference/` but is frozen history, so it is not migrated.
- Once ball-balancer no longer needs it, the Sandcastle `reference/` mount is removed. `CONTEXT.md` describes the golden source as what projects read and copy from.
- This settles how a project reads from `reference/`. ADR-0004's promotion, which writes back into `reference/`, is unchanged.
