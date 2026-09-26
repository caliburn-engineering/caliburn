// Factory Floor — log ingestion.
//
// Sandcastle's file logger is the only artefact every AFK run leaves behind
// regardless of entrypoint, so it is the source of truth here rather than a
// hook inside main.ts. Nothing in .sandcastle/ has to change, a run already in
// flight is picked up mid-write, and a run launched from a phone six hours ago
// still lands in the UI.
//
// The trade that buys: the log carries no per-event timestamps, so the trace
// orders events by sequence and times only the phases Sandcastle itself times.

import { execFileSync } from "node:child_process";
import { readFileSync, readdirSync, statSync } from "node:fs";
import { basename, join, resolve } from "node:path";
import type { Db } from "./db.js";

export interface ParsedPhase {
  seq: number;
  iteration: number | null;
  name: string;
  kind: "setup" | "iteration" | "wrapup";
  status: "ok" | "running" | "fail";
  durationMs: number | null;
  detail: string | null;
}

export interface ParsedEvent {
  seq: number;
  iteration: number | null;
  phaseSeq: number | null;
  type: "text" | "tool" | "skill" | "note" | "report";
  name: string | null;
  detail: string;
}

export interface ParsedRun {
  runId: string;
  name: string | null;
  issue: number | null;
  harness: string | null;
  sandbox: string | null;
  branch: string | null;
  status: "running" | "complete" | "stopped" | "failed" | "unknown";
  maxIterations: number | null;
  iterations: number;
  completed: boolean;
  /** ended_at came from the next run's start, so it is an upper bound only. */
  endBounded: boolean;
  toolCalls: number;
  summary: string | null;
  startedAt: string | null;
  endedAt: string | null;
  phases: ParsedPhase[];
  events: ParsedEvent[];
}

const RUN_START = /^--- Run started: (.+) ---$/;
const FIELD = /^ {2}(Agent|Sandbox|Max iterations|Branch): (.+)$/;
const ITERATION = /^Iteration (\d+)\/(\d+)$/;
const PHASE_DONE = /^(.+?) done \(([\d.]+)s\)$/;
const PHASE_START = /^(Copying to worktree|Setting up sandbox|Expanding shell expressions|Collecting commits)\.{0,3}$/;
const TOOL_OPEN = /^([A-Z][A-Za-z0-9_]*)\((.*)$/;
const COMPLETION = /^Agent signaled completion after (\d+) iteration/;
const RUN_END = /^Run (complete|failed|stopped)\b:?\s*(.*)$/;
const SKILL_PATH = /(?:\.claude|\.pi)\/skills\/([a-z0-9][a-z0-9-]*)/i;
const PROMISE = /<promise>\s*([^<]*?)\s*<\/promise>/gi;
const PROMISE_ONLY = /^<promise>\s*([^<]*?)\s*<\/promise>$/i;
const ISSUE_WRITE = /^gh issue (close|comment)\s+(\d+)/;
/** Sandcastle prints the harness's own failure verbatim; a rate limit is the common one. */
const AGENT_FAILED = /^Agent invocation failed|hit your session limit|rate.?limit/i;

/**
 * Which issue a run is working. Three sources, because run naming is a habit
 * rather than a contract: the run name ("ticket-44", "afk-39"), the branch
 * ("sc/44-guardrail"), and failing both, the first `gh issue view N` the agent
 * runs. Any trailing number in a name is taken as the issue — that is what every
 * naming scheme used here has meant so far.
 */
function issueFromName(name: string): number | null {
  const tail = /(?:^|[^0-9])(\d+)\s*$/.exec(name.trim());
  if (tail) return Number(tail[1]);
  const labelled = /(?:afk|issue|ticket|sc)[-#_ ]?(\d+)/i.exec(name);
  return labelled ? Number(labelled[1]) : null;
}

function issueFromBranch(branch: string): number | null {
  const m = /(?:^|[/-])(\d+)(?:[-/]|$)/.exec(branch);
  return m ? Number(m[1]) : null;
}

function issueFromCommand(text: string): number | null {
  const m = /gh issue (?:view|close|comment|edit)\s+(\d+)/.exec(text);
  return m ? Number(m[1]) : null;
}

/**
 * A skill shows up two ways: Claude Code's own Skill tool, or the agent reading
 * a SKILL.md out of the mounted ~/.claude/skills tree. Both are worth a lane.
 */
export function detectSkill(tool: string, args: string): string | null {
  if (tool === "Skill") {
    const named = /^\s*([a-z0-9][a-z0-9:-]*)/i.exec(args);
    return named ? named[1] : null;
  }
  const path = SKILL_PATH.exec(args);
  return path ? path[1] : null;
}

/**
 * The comment body as typed is wrapped in shell scaffolding — `--comment "$(cat
 * <<'EOF' … EOF)"`. Strip that so the panel shows what a reader would see on the
 * ticket, not the plumbing that delivered it.
 */
function cleanReport(args: string): string {
  const heredoc = /<<'?(\w+)'?\s*\n([\s\S]*?)\n\s*\1/.exec(args);
  if (heredoc) return heredoc[2].trim();
  const flag = /(?:--comment|--body|-c|-b)\s+(["'])([\s\S]*)\1\s*$/.exec(args);
  if (flag) return flag[2].trim();
  return args.replace(ISSUE_WRITE, "").trim();
}

/**
 * A tool call can span lines (heredocs, line continuations), so consume until
 * the parens balance. Depth starts at 1 because TOOL_OPEN has already eaten the
 * opening paren — starting at 0 closes every call on its first line.
 */
function readToolCall(lines: string[], start: number): { args: string; next: number } {
  let depth = 1;
  let args = "";
  for (let i = start; i < lines.length; i++) {
    const line = i === start ? lines[i].replace(TOOL_OPEN, "$2") : lines[i];
    for (const ch of line) {
      if (ch === "(") depth++;
      else if (ch === ")") depth--;
    }
    args += (i === start ? "" : "\n") + line;
    if (depth <= 0) return { args: args.replace(/\)\s*$/, ""), next: i + 1 };
    if (i - start > 200) break; // a runaway heredoc should not eat the file
  }
  return { args, next: start + 1 };
}

/**
 * One log file can hold several runs — Sandcastle appends, and a re-run under
 * the same name reopens the same path. Each "Run started" banner opens a new one.
 */
export function parseLog(text: string, logPath: string, liveCutoffMs = 10 * 60_000): ParsedRun[] {
  const lines = text.split("\n");
  const runs: ParsedRun[] = [];
  let run: ParsedRun | null = null;
  let seq = 0;
  let phaseSeq = 0;
  let iteration: number | null = null;
  let inAgent = false;
  let lastText: ParsedEvent | null = null;

  const push = (e: Omit<ParsedEvent, "seq">) => {
    if (!run) return;
    const event: ParsedEvent = { ...e, seq: seq++ };
    run.events.push(event);
    return event;
  };

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    const trimmed = line.trimEnd();

    const started = RUN_START.exec(trimmed);
    if (started) {
      run = {
        runId: `${basename(logPath, ".log")}#${started[1]}`,
        name: null, issue: null, harness: null, sandbox: null, branch: null,
        status: "unknown", maxIterations: null, iterations: 0, completed: false,
        endBounded: false, toolCalls: 0, summary: null, startedAt: started[1], endedAt: null,
        phases: [], events: [],
      };
      runs.push(run);
      seq = 0; phaseSeq = 0; iteration = null; inAgent = false; lastText = null;
      continue;
    }
    if (!run) continue;

    const field = FIELD.exec(trimmed);
    if (field) {
      const value = field[2].trim();
      if (field[1] === "Agent") { run.name = value; run.issue = issueFromName(value); }
      else if (field[1] === "Sandbox") run.sandbox = value;
      else if (field[1] === "Branch") { run.branch = value; run.issue = run.issue ?? issueFromBranch(value); }
      else if (field[1] === "Max iterations") run.maxIterations = Number(value);
      continue;
    }

    const iter = ITERATION.exec(trimmed);
    if (iter) {
      iteration = Number(iter[1]);
      run.iterations = Math.max(run.iterations, iteration);
      run.phases.push({
        seq: phaseSeq++, iteration, name: `Iteration ${iteration}`, kind: "iteration",
        status: "running", durationMs: null, detail: null,
      });
      lastText = null;
      continue;
    }

    if (PHASE_START.test(trimmed)) {
      const label = trimmed.replace(/\.{3}$/, "");
      run.phases.push({
        seq: phaseSeq++, iteration, name: label,
        kind: label === "Collecting commits" ? "wrapup" : "setup",
        status: "running", durationMs: null, detail: null,
      });
      continue;
    }

    const done = PHASE_DONE.exec(trimmed);
    if (done && !inAgent) {
      const name = done[1].replace(/\.{3}$/, "");
      const phase = [...run.phases].reverse().find((p) => p.name === name);
      if (phase) {
        phase.status = "ok";
        phase.durationMs = Math.round(Number(done[2]) * 1000);
      }
      continue;
    }

    if (trimmed === "Agent started") { inAgent = true; lastText = null; continue; }
    if (trimmed === "Agent stopped") {
      inAgent = false;
      const active = [...run.phases].reverse().find((p) => p.kind === "iteration" && p.status === "running");
      if (active) active.status = "ok";
      continue;
    }

    const completion = COMPLETION.exec(trimmed);
    if (completion) {
      run.completed = true;
      run.iterations = Number(completion[1]);
      continue;
    }

    const ended = RUN_END.exec(trimmed);
    if (ended) {
      if (run.status !== "failed") {
        run.status = ended[1] === "complete" ? "complete" : ended[1] === "stopped" ? "stopped" : "failed";
      }
      push({ iteration, phaseSeq: null, type: "note", name: `Run ${ended[1]}`, detail: ended[2] ?? "" });
      continue;
    }

    // Prompt expansion lines name the issue the run is about to work.
    if (!inAgent && /→ ~\d+ tokens$/.test(trimmed)) {
      const issue = issueFromCommand(trimmed);
      if (issue && !run.issue) run.issue = issue;
      push({ iteration, phaseSeq: phaseSeq - 1, type: "note", name: "prompt", detail: trimmed.trim() });
      continue;
    }

    if (!inAgent || trimmed.trim() === "") continue;

    const tool = TOOL_OPEN.exec(trimmed);
    if (tool) {
      const { args, next } = readToolCall(lines, i);
      i = next - 1;
      run.toolCalls++;
      run.issue = run.issue ?? issueFromCommand(args);
      push({ iteration, phaseSeq: phaseSeq - 1, type: "tool", name: tool[1], detail: args.trim() });

      // prompt.md tells the agent to explain itself in a comment when it closes
      // an issue or gives up on one. That comment — not the prose around it — is
      // usually where the conclusion and any question back to a human live.
      const write = ISSUE_WRITE.exec(args.trim());
      if (write) {
        push({
          iteration, phaseSeq: phaseSeq - 1, type: "report",
          name: write[1] === "close" ? `closed #${write[2]}` : `commented on #${write[2]}`,
          detail: cleanReport(args),
        });
      }
      const skill = detectSkill(tool[1], args);
      if (skill) push({ iteration, phaseSeq: phaseSeq - 1, type: "skill", name: skill, detail: tool[1] });
      lastText = null;
      continue;
    }

    // The completion signal arrives as a text chunk. Left as prose it wins the
    // "last thing said" contest and the conclusion panel reads <promise>COMPLETE.
    if (AGENT_FAILED.test(trimmed.trim())) {
      run.status = "failed";
      push({ iteration, phaseSeq: phaseSeq - 1, type: "note", name: "failed", detail: trimmed.trim() });
      lastText = null;
      continue;
    }

    const promise = PROMISE_ONLY.exec(trimmed.trim());
    if (promise) {
      push({ iteration, phaseSeq: phaseSeq - 1, type: "note", name: "promise", detail: promise[1] });
      lastText = null;
      continue;
    }

    // Consecutive prose belongs to one thought; the logger breaks it by chunk.
    if (lastText && lastText.detail.length < 4000) {
      lastText.detail += "\n" + trimmed;
    } else {
      lastText = push({ iteration, phaseSeq: phaseSeq - 1, type: "text", name: null, detail: trimmed }) ?? null;
    }
  }

  const stats = statSync(logPath);
  const ageMs = Date.now() - stats.mtimeMs;
  for (const [index, r] of runs.entries()) {
    const last = index === runs.length - 1;
    if (r.status === "unknown" && last && ageMs < liveCutoffMs) {
      r.status = "running";
    }
    if (r.status !== "running") {
      // The log carries no end timestamp. For the last run in a file, the file's
      // own mtime is the best bound; for an earlier one it is badly wrong — a
      // re-run hours later would stretch it to match. The next run's start is the
      // tightest bound that is actually true.
      if (last) {
        r.endedAt = r.endedAt ?? new Date(stats.mtimeMs).toISOString();
      } else {
        r.endedAt = r.endedAt ?? runs[index + 1].startedAt ?? new Date(stats.mtimeMs).toISOString();
        r.endBounded = true;
      }
    }
    // The agent's closing words are what a human reads first: what it decided,
    // and anything it is asking back. Keep them whole rather than truncated —
    // for a run still going, this is simply its most recent thought.
    //
    // "Last text event" is not enough on its own: the tail of a run is littered
    // with one-line acks ("Done.", a stray tag), so the last SUBSTANTIAL block
    // wins, and any inline completion tag is stripped out of it.
    const prose = [...r.events]
      .reverse()
      .filter((e) => e.type === "text")
      .map((e) => e.detail.replace(PROMISE, "").trim());
    r.summary = (prose.find((text) => text.length >= 40) ?? prose[0] ?? null)?.slice(0, 4000) ?? null;
    r.harness = r.harness ?? "claude";
  }
  return runs;
}

/**
 * The log does not print commit SHAs, so they are recovered from git by time
 * window. Best effort: a failure here must never cost us the run.
 */
function commitsFor(run: ParsedRun, repoRoot: string): Array<{ sha: string; subject: string; at: string }> {
  if (!run.startedAt) return [];
  try {
    const out = execFileSync(
      "git",
      ["log", "--all", "--since", run.startedAt, ...(run.endedAt ? ["--until", run.endedAt] : []),
       "--format=%H%x1f%s%x1f%cI"],
      { cwd: repoRoot, encoding: "utf8", timeout: 5000 },
    );
    return out.split("\n").filter(Boolean).map((line) => {
      const [sha, subject, at] = line.split("\x1f");
      return { sha, subject, at };
    }).filter((c) => /^RALPH/i.test(c.subject) || (run.issue !== null && c.subject.includes(`#${run.issue}`)));
  } catch {
    return [];
  }
}

/** Re-importing a log replaces every run it holds — parsing is the only truth. */
export function importLog(db: Db, logPath: string, repoRoot: string): ParsedRun[] {
  const stats = statSync(logPath);
  const runs = parseLog(readFileSync(logPath, "utf8"), logPath);

  const oldIds = db.prepare("SELECT run_id FROM runs WHERE log_path = ?").all(logPath) as Array<{ run_id: string }>;
  for (const { run_id } of oldIds) {
    db.prepare("DELETE FROM events WHERE run_id = ?").run(run_id);
    db.prepare("DELETE FROM phases WHERE run_id = ?").run(run_id);
    db.prepare("DELETE FROM commits WHERE run_id = ?").run(run_id);
  }
  db.prepare("DELETE FROM runs WHERE log_path = ?").run(logPath);

  const insertRun = db.prepare(
    `INSERT INTO runs (run_id, name, issue, harness, sandbox, branch, status, max_iterations,
      iterations, completed, tool_calls, log_path, log_size, summary, started_at, ended_at, end_bounded)
     VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)`,
  );
  const insertPhase = db.prepare(
    `INSERT INTO phases (phase_id, run_id, seq, iteration, name, kind, status, duration_ms, detail)
     VALUES (?,?,?,?,?,?,?,?,?)`,
  );
  const insertEvent = db.prepare(
    "INSERT INTO events (run_id, phase_id, iteration, seq, type, name, detail) VALUES (?,?,?,?,?,?,?)",
  );
  const insertCommit = db.prepare(
    "INSERT OR REPLACE INTO commits (run_id, sha, subject, issue, committed_at) VALUES (?,?,?,?,?)",
  );

  for (const run of runs) {
    insertRun.run(
      run.runId, run.name, run.issue, run.harness, run.sandbox, run.branch, run.status,
      run.maxIterations, run.iterations, run.completed ? 1 : 0, run.toolCalls,
      logPath, stats.size, run.summary, run.startedAt, run.endedAt, run.endBounded ? 1 : 0,
    );
    for (const p of run.phases) {
      insertPhase.run(
        `${run.runId}:${p.seq}`, run.runId, p.seq, p.iteration, p.name, p.kind, p.status, p.durationMs, p.detail,
      );
    }
    for (const e of run.events) {
      insertEvent.run(
        run.runId, e.phaseSeq === null ? null : `${run.runId}:${e.phaseSeq}`,
        e.iteration, e.seq, e.type, e.name, e.detail,
      );
    }
    for (const c of commitsFor(run, repoRoot)) {
      insertCommit.run(run.runId, c.sha, c.subject, run.issue, c.at);
    }
  }
  return runs;
}

/**
 * Re-import only the logs that grew.
 *
 * The seen-log table is what makes this cheap: a `*.console.log` holds no run
 * banner, so keying "have I read this?" off the runs table would re-parse it on
 * every single tick. Sandcastle's own console logs are skipped by name as well —
 * they are orchestrator stdout, already summarised by the run they belong to.
 */
export function syncLogs(db: Db, logDir: string, repoRoot: string): number {
  let changed = 0;
  let entries: string[];
  try {
    entries = readdirSync(logDir).filter((f) => f.endsWith(".log") && !f.endsWith(".console.log"));
  } catch {
    return 0;
  }
  const seen = db.prepare("SELECT size, mtime_ms FROM log_files WHERE path = ?");
  const remember = db.prepare(
    "INSERT OR REPLACE INTO log_files (path, size, mtime_ms, runs) VALUES (?,?,?,?)",
  );
  const stillRunning = db.prepare(
    "SELECT COUNT(*) AS n FROM runs WHERE log_path = ? AND status = 'running'",
  );

  for (const entry of entries) {
    const path = resolve(join(logDir, entry));
    const stats = statSync(path);
    const known = seen.get(path) as { size: number; mtime_ms: number } | undefined;
    const unchanged = known && known.size === stats.size && known.mtime_ms === Math.floor(stats.mtimeMs);
    // A run last seen as "running" is re-read even when the file has not grown,
    // so a finished-but-quiet run stops claiming to be live.
    const live = (stillRunning.get(path) as { n: number }).n > 0;
    if (unchanged && !live) continue;

    const runs = importLog(db, path, repoRoot);
    remember.run(path, stats.size, Math.floor(stats.mtimeMs), runs.length);
    changed++;
  }
  return changed;
}
