// Run ONE ready-for-agent issue in its own sandcastle run, on its own branch.
//
//   npx tsx .sandcastle/ticket.ts <issue-number>
//
// Unlike main.ts (which grabs the highest-priority unblocked issue and works the
// Caliburn repo), this pins the agent to a single issue and can anchor the run
// in a project repo under projects/ via sandcastle's `cwd`. Each ticket gets its
// own branch (sc/<n>-<slug>) and its own log, so several can be reviewed apart.
//
// Commits land on the ticket's local branch. Nothing is pushed and no issue is
// closed here — a human pushes the branch and opens the PR; the issue closes on
// merge. That keeps a failed or half-done run from closing its own issue.

import { run } from "@ai-hero/sandcastle";
import { afkLogging, agents, caliburnSandbox, notify } from "./caliburn.js";

const ISSUE_REPO = "caliburn-engineering/caliburn"; // issues live here for every project

// Which repo each ticket's CODE lives in, and a short branch slug.
//  - "caliburn": worked in this repo (default cwd).
//  - "bb": anchored at projects/ball-balancer (its own git repo). projects/ is
//    gitignored in Caliburn, so the run MUST be anchored there to see the code.
const TICKETS: Record<number, { repo: "caliburn" | "bb"; slug: string }> = {
  41: { repo: "caliburn", slug: "housekeeping" },
  40: { repo: "bb", slug: "ci" },
  42: { repo: "bb", slug: "stab" },
  43: { repo: "bb", slug: "oracle" },
  44: { repo: "bb", slug: "guardrail" },
  45: { repo: "bb", slug: "care-hardening" },
  46: { repo: "bb", slug: "guardrail-fixes" },
  47: { repo: "bb", slug: "impulse" },
  48: { repo: "bb", slug: "locus-labels" },
  49: { repo: "bb", slug: "nyquist" },
  50: { repo: "bb", slug: "fixture-compare" },
  51: { repo: "bb", slug: "residual-gate" },
  52: { repo: "bb", slug: "live-oracle" },
  53: { repo: "bb", slug: "pydrake" },
  63: { repo: "bb", slug: "roundoff-robust" },
  64: { repo: "bb", slug: "nominal-pumping" },
};

const n = Number(process.argv[2]);
const meta = TICKETS[n];
if (!meta) {
  console.error(`Unknown or unmapped issue: ${process.argv[2]}`);
  console.error(`Known: ${Object.keys(TICKETS).join(", ")}`);
  process.exit(1);
}

const branch = `sc/${n}-${meta.slug}`;
const logName = `ticket-${n}`;
const isBB = meta.repo === "bb";

// Verification differs by repo. The AFK image has the C++/Eigen toolchain the
// reference/ tree needs; a browser (Emscripten) toolchain and desktop GL are NOT
// assumed present, so the ball-balancer path verifies the DESKTOP test suite
// only (which is what this workstream's CI does too) and builds test targets
// rather than the GL visualizer when GL is absent.
const verify = isBB
  ? `Build and run the desktop test suite:
    cmake -S . -B build -DGLFW_BUILD_WAYLAND=OFF && cmake --build build -j && ctest --test-dir build --output-on-failure
  The image has no Wayland dev libraries, hence -DGLFW_BUILD_WAYLAND=OFF (CI installs
  them and needs no flag). The first configure fetches Eigen and takes several
  minutes — do not abandon it.
  If the full build fails only because the GL 'visualizer' target needs a display
  library the container lacks, build and test the affected library and test
  targets instead (e.g. --target test_<name>) and say so in your issue comment.
  Do NOT attempt the Emscripten/web build; this workstream's CI is desktop-only.`
  : `Verify per what you changed, following AGENTS.md and docs/agents:
    - C++ in reference/: cmake -B reference/build -S reference && cmake --build reference/build && ctest --test-dir reference/build --output-on-failure
    - Docs only: no build; confirm every link resolves to a file that exists.
    - New GitHub issues you create: confirm they exist with the intended labels.`;

const prompt = `You are RALPH, working a SINGLE pre-selected issue. Do NOT look for
other work and do NOT run an unfiltered issue query.

# The one issue

Work issue #${n}, and only #${n}. Read it in full first:

  gh issue view ${n} --repo ${ISSUE_REPO} --comments

All \`gh\` issue commands in this run target \`--repo ${ISSUE_REPO}\` explicitly —
the issues for every project live in that one tracker, not in the project repo
you are anchored in.

# Repo orientation

You are anchored in ${isBB ? "the ball-balancer project repo" : "the Caliburn workspace repo"}.
Read \`AGENTS.md\` first, then \`CONTEXT.md\` and any ADRs under \`docs/adr/\` that
touch your area (${isBB ? "ball-balancer keeps its ubiquitous language in CONTEXT.md" : "the workspace CONTEXT.md and docs/adr/ hold the architecture"}).
Respect every ADR; if your change would contradict one, stop and say so in an
issue comment instead of overriding it silently.

# Workflow

1. Explore — read the issue and the source and tests it names before writing code.
   If the issue's body has a "Blocked by: #N" line naming an OPEN issue, stop and
   comment that it is blocked; do not start it.
2. Plan — the smallest change that satisfies the acceptance criteria.
3. Execute — red → green → refactor: a failing test first where the issue is
   testable, then the implementation.
4. Verify — ${verify}
   Fix failures before continuing.
5. Commit — ONE commit on this run's branch (${branch}). The message must start
   with "RALPH:", state the issue number, list key decisions, and list files
   changed.
6. Comment, do NOT close — post a \`gh issue comment ${n} --repo ${ISSUE_REPO}\`
   summarising what you did and naming the branch \`${branch}\`. Do NOT close the
   issue: a human reviews the branch as a pull request, and the issue closes when
   that merges.

# Rules

- One issue only. Never touch another issue's scope.
- Never invent a citation; if you cannot source a claim, leave it out and say so.
- No commented-out code, no TODOs left in committed code.
- If you are blocked (missing context, a failure you cannot fix), comment on the
  issue explaining the blocker and stop — do not force a commit.

# Done

When the issue is committed and verified (or you are blocked and have commented),
output the completion signal:

<promise>COMPLETE</promise>`;

notify("Sandcastle", `ticket #${n} starting on ${branch}`);

const result = await run({
  name: `ticket-${n}`,
  sandbox: caliburnSandbox(),
  agent: agents.claude,
  ...(isBB ? { cwd: "projects/ball-balancer" } : {}),
  prompt,
  maxIterations: 3,
  branchStrategy: { type: "branch", branch },
  idleTimeoutSeconds: 1800,
  // Caliburn has a TypeScript toolchain worth warming; ball-balancer is pure C++
  // with no package.json, so copying node_modules and running npm install there
  // is both useless and a failure waiting to happen.
  ...(isBB
    ? {}
    : {
        copyToWorktree: ["node_modules"],
        hooks: { sandbox: { onSandboxReady: [{ command: "npm install", timeoutMs: 300_000 }] } },
      }),
  logging: afkLogging(logName),
});

const summary = result.commits.length
  ? `#${n}: ${result.commits.length} commit(s) on ${branch} over ${result.iterations.length} iteration(s)`
  : `#${n}: no commits after ${result.iterations.length} iteration(s) — check the issue for a blocker comment`;

console.log(`\n${summary}`);
console.log(`Review with: git ${isBB ? "-C projects/ball-balancer " : ""}log --oneline -5 ${branch}`);
notify("Sandcastle finished", summary);
