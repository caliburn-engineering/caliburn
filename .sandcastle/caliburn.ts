// Shared Sandcastle configuration for Caliburn.
//
// Both smoke.ts and main.ts import from here so the sandbox contract — image,
// mounts, notification bridge — is defined exactly once.

import { spawn } from "node:child_process";
import { docker } from "@ai-hero/sandcastle/sandboxes/docker";
import { claudeCode, pi } from "@ai-hero/sandcastle";

/**
 * Matt Pocock's engineering skills live at ~/.claude/skills — USER level, outside
 * any repo. A fresh container has no such directory, so without this mount the
 * agent loses wayfinder, triage, to-tickets, implement, code-review and the rest.
 *
 * Caliburn's own .pi/skills/ need no mount: they are tracked in git and arrive
 * with the worktree. (Except .pi/skills/local-* which is gitignored and will not
 * reach a worktree-based run.)
 */
export const caliburnSandbox = () =>
  docker({
    imageName: "sandcastle:caliburn",
    mounts: [
      {
        hostPath: "~/.claude/skills",
        sandboxPath: "/home/agent/.claude/skills",
        readonly: true,
      },
    ],
  });

/** Swap harnesses here. Both CLIs are installed in the image. */
export const agents = {
  pi: pi("claude-sonnet-4-6", { thinking: "high" }),
  // captureSessions: false works around a Sandcastle 0.12.0 bug — after the
  // agent finishes, its `docker cp` of the Claude Code session .jsonl fails
  // (SessionCaptureError, file not found at the expected encoded path) and
  // crashes the whole run even though the agent's work succeeded. None of our
  // runs pass resumeSession/forkSession, so this loses nothing for us today.
  claude: claudeCode("claude-sonnet-4-6", {
    effort: "high",
    captureSessions: false,
  }),
};

/** Fire the host-side notifier (notify-send + TTS routed to phone or desktop). */
export function notify(title: string, message: string): void {
  spawn(".sandcastle/notify.sh", [title, message], {
    stdio: "ignore",
    detached: true,
  }).unref();
}

/**
 * Logging config that keeps a phone informed of a run it cannot watch.
 *
 * onAgentStreamEvent fires for every text chunk and tool call, which is far too
 * chatty to notify on directly — so we announce iteration boundaries only.
 */
export function afkLogging(name: string) {
  let lastIteration = 0;
  return {
    type: "file" as const,
    path: `.sandcastle/logs/${name}.log`,
    onAgentStreamEvent: (event: { iteration: number }) => {
      if (event.iteration > lastIteration) {
        lastIteration = event.iteration;
        notify("Sandcastle", `${name}: iteration ${event.iteration} started`);
      }
    },
  };
}
