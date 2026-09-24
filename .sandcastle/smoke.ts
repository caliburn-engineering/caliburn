// Tracer bullet: verify the sandbox is actually Caliburn-shaped before trusting
// it with an unattended issue loop.
//
//   npx tsx .sandcastle/smoke.ts
//
// Read-only and single-iteration. It checks the things that are easy to get
// wrong and fail silently: the pi package name, the ~/.claude/skills mount, and
// the C++ toolchain the reference/ tree needs.

import { run, Output } from "@ai-hero/sandcastle";
import { agents, caliburnSandbox, notify } from "./caliburn.js";

const result = await run({
  name: "smoke",
  sandbox: caliburnSandbox(),
  agent: agents.claude,
  promptFile: ".sandcastle/smoke-prompt.md",
  maxIterations: 1,
  branchStrategy: { type: "branch", branch: "sandcastle/smoke" },
  logging: { type: "stdout" },
  output: Output.string({ tag: "report" }),
});

console.log("\n=== SMOKE REPORT ===\n");
console.log(result.output ?? "(no report emitted)");

notify("Sandcastle smoke", result.output ? "completed" : "no report emitted");
