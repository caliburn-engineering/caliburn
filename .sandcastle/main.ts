// Caliburn AFK loop: work `ready-for-agent` issues one at a time, unattended.
//
//   npx tsx .sandcastle/main.ts        (or: work caliburn afk)
//
// Commits land on the `sandcastle/afk` branch, NOT on main — an unattended run
// launched from a phone should never write straight to the default branch.
// Review with `git log main..sandcastle/afk` and merge by hand.

import { run } from "@ai-hero/sandcastle";
import { afkLogging, agents, caliburnSandbox, notify } from "./caliburn.js";

const BRANCH = "sandcastle/afk";

notify("Sandcastle", "AFK run starting");

const result = await run({
  name: "afk",
  sandbox: caliburnSandbox(),

  // Using the Claude subscription (CLAUDE_CODE_OAUTH_TOKEN) rather than pi's
  // metered ANTHROPIC_API_KEY. Both CLIs are installed in the image already.
  agent: agents.claude,

  promptFile: ".sandcastle/prompt.md",

  // Each iteration works a single issue.
  maxIterations: 3,

  branchStrategy: { type: "branch", branch: BRANCH },

  // Copy host node_modules in for fast startup; the sandbox hook below is the
  // safety net for platform-specific binaries.
  copyToWorktree: ["node_modules"],

  // A C++ build that fetches Eigen from scratch can easily exceed the 10-minute
  // default while producing no output.
  idleTimeoutSeconds: 1800,

  hooks: {
    sandbox: {
      onSandboxReady: [{ command: "npm install", timeoutMs: 300_000 }],
    },
  },

  logging: afkLogging("afk"),
});

const summary = result.commits.length
  ? `${result.commits.length} commit(s) on ${BRANCH} over ${result.iterations.length} iteration(s)`
  : `no commits after ${result.iterations.length} iteration(s)`;

console.log(`\n${summary}`);
console.log(`Review with: git log main..${BRANCH}`);

notify("Sandcastle finished", summary);
