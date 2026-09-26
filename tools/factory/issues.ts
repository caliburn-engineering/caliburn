// Factory Floor — the issue board.
//
// The tracker is GitHub Issues (docs/agents/issue-tracker.md) and the five
// canonical triage labels are the columns (docs/agents/triage-labels.md). The
// board adds one state GitHub cannot know: an issue a Sandcastle run is working
// RIGHT NOW, which comes from the trace db rather than from a label.
//
// Closing an issue drops its triage label by convention, so "done" is read from
// GitHub state and never from a label.

import { execFile } from "node:child_process";
import { promisify } from "node:util";
import type { Db } from "./db.js";

const run = promisify(execFile);

export type Column =
  | "in-flight" | "ready-for-agent" | "needs-triage" | "needs-info"
  | "ready-for-human" | "done" | "wontfix" | "unlabelled";

export interface Issue {
  number: number;
  title: string;
  /** The ticket text, trimmed — enough to see what the work actually is. */
  body: string;
  state: "OPEN" | "CLOSED";
  url: string;
  labels: string[];
  wayfinder: string[];
  blockedBy: number[];
  column: Column;
  updatedAt: string;
  closedAt: string | null;
  /** run_id of the AFK run touching this issue, when one is live. */
  activeRun: string | null;
  /** SHAs committed against this issue by any traced run. */
  commits: Array<{ sha: string; subject: string; runId: string }>;
}

interface GhIssue {
  number: number;
  title: string;
  state: string;
  url: string;
  body: string;
  updatedAt: string;
  closedAt: string | null;
  labels: Array<{ name: string }>;
}

const TRIAGE: Array<[string, Column]> = [
  ["ready-for-agent", "ready-for-agent"],
  ["needs-info", "needs-info"],
  ["ready-for-human", "ready-for-human"],
  ["needs-triage", "needs-triage"],
];

let cache: { at: number; issues: Issue[] } | null = null;
let inFlight: Promise<Issue[]> | null = null;
const TTL_MS = 30_000;

export interface Comment {
  author: string;
  body: string;
  createdAt: string;
  url: string;
}

export interface Thread {
  number: number;
  state: "OPEN" | "CLOSED";
  closedAt: string | null;
  comments: Comment[];
}

/**
 * The ticket thread, live from GitHub.
 *
 * Separate from the board fetch on purpose: comments are only wanted for the one
 * issue being looked at, and pulling every body for 50+ issues to show one thread
 * would be a waste on every poll. Cached per issue, briefly — a reply you just
 * wrote should show up while you are still looking at the page.
 */
const threads = new Map<number, { at: number; thread: Thread }>();
const threadsInFlight = new Map<number, Promise<Thread>>();
const THREAD_TTL_MS = 20_000;

export async function fetchThread(repoRoot: string, number: number, force = false): Promise<Thread> {
  const cached = threads.get(number);
  if (!force && cached && Date.now() - cached.at < THREAD_TTL_MS) return cached.thread;
  const pending = threadsInFlight.get(number);
  if (pending) return pending;

  const work = (async () => {
    const { stdout } = await run(
      "gh",
      ["issue", "view", String(number), "--json", "number,state,closedAt,comments"],
      { cwd: repoRoot, maxBuffer: 8 * 1024 * 1024 },
    );
    const raw = JSON.parse(stdout) as {
      number: number;
      state: string;
      closedAt: string | null;
      comments: Array<{ author?: { login?: string }; body: string; createdAt: string; url: string }>;
    };
    const thread: Thread = {
      number: raw.number,
      state: raw.state === "CLOSED" ? "CLOSED" : "OPEN",
      closedAt: raw.closedAt,
      comments: (raw.comments ?? []).map((c) => ({
        author: c.author?.login ?? "unknown",
        body: (c.body ?? "").trim(),
        createdAt: c.createdAt,
        url: c.url,
      })),
    };
    threads.set(number, { at: Date.now(), thread });
    return thread;
  })();

  threadsInFlight.set(number, work);
  try {
    return await work;
  } finally {
    threadsInFlight.delete(number);
  }
}

/** "Blocked by: #12" in the body is the convention prompt.md already reads. */
function blockedRefs(body: string): number[] {
  const out = new Set<number>();
  const re = /blocked by:?\s*((?:#\d+[,\s]*)+)/gi;
  let m: RegExpExecArray | null;
  while ((m = re.exec(body))) {
    for (const ref of m[1].match(/\d+/g) ?? []) out.add(Number(ref));
  }
  return [...out];
}

function classify(gh: GhIssue, labels: string[], activeRun: string | null): Column {
  if (gh.state === "CLOSED") return labels.includes("wontfix") ? "wontfix" : "done";
  if (activeRun) return "in-flight";
  for (const [label, column] of TRIAGE) if (labels.includes(label)) return column;
  return "unlabelled";
}

/** One issue by number, for the run detail pane. Rides the same 30s cache. */
export async function issueByNumber(db: Db, repoRoot: string, number: number): Promise<Issue | null> {
  try {
    const issues = await fetchIssues(db, repoRoot);
    return issues.find((i) => i.number === number) ?? null;
  } catch {
    return null;
  }
}

/** number → title, for labelling the run list without a second gh call. */
export async function issueTitles(db: Db, repoRoot: string): Promise<Map<number, string>> {
  try {
    const issues = await fetchIssues(db, repoRoot);
    return new Map(issues.map((i) => [i.number, i.title]));
  } catch {
    return new Map();
  }
}

export async function fetchIssues(db: Db, repoRoot: string, force = false): Promise<Issue[]> {
  if (!force && cache && Date.now() - cache.at < TTL_MS) return cache.issues;
  if (inFlight) return inFlight;

  inFlight = (async () => {
    const { stdout } = await run(
      "gh",
      ["issue", "list", "--state", "all", "--limit", "300",
       "--json", "number,title,state,url,body,updatedAt,closedAt,labels"],
      { cwd: repoRoot, maxBuffer: 16 * 1024 * 1024 },
    );
    const raw = JSON.parse(stdout) as GhIssue[];

    // An issue is "in flight" when a run that is still going names it.
    const live = db.prepare("SELECT run_id, issue FROM runs WHERE status = 'running' AND issue IS NOT NULL")
      .all() as Array<{ run_id: string; issue: number }>;
    const liveByIssue = new Map(live.map((r) => [r.issue, r.run_id]));

    const commitRows = db.prepare(
      "SELECT run_id, sha, subject, issue FROM commits WHERE issue IS NOT NULL ORDER BY committed_at",
    ).all() as Array<{ run_id: string; sha: string; subject: string; issue: number }>;
    const commitsByIssue = new Map<number, Array<{ sha: string; subject: string; runId: string }>>();
    for (const c of commitRows) {
      const list = commitsByIssue.get(c.issue) ?? [];
      list.push({ sha: c.sha.slice(0, 7), subject: c.subject, runId: c.run_id });
      commitsByIssue.set(c.issue, list);
    }

    const issues: Issue[] = raw.map((gh) => {
      const labels = gh.labels.map((l) => l.name);
      const activeRun = liveByIssue.get(gh.number) ?? null;
      return {
        number: gh.number,
        title: gh.title,
        body: (gh.body ?? "").trim().slice(0, 6000),
        state: gh.state === "CLOSED" ? "CLOSED" : "OPEN",
        url: gh.url,
        labels: labels.filter((l) => !l.startsWith("wayfinder:")),
        wayfinder: labels.filter((l) => l.startsWith("wayfinder:")).map((l) => l.slice("wayfinder:".length)),
        blockedBy: blockedRefs(gh.body ?? ""),
        column: classify(gh, labels, activeRun),
        updatedAt: gh.updatedAt,
        closedAt: gh.closedAt,
        activeRun,
        commits: commitsByIssue.get(gh.number) ?? [],
      };
    });

    // An issue whose blocker is still open cannot be picked up, and the board
    // should say so rather than leaving it looking available.
    const openNumbers = new Set(issues.filter((i) => i.state === "OPEN").map((i) => i.number));
    for (const issue of issues) {
      issue.blockedBy = issue.blockedBy.filter((n) => openNumbers.has(n));
    }

    cache = { at: Date.now(), issues };
    return issues;
  })();

  try {
    return await inFlight;
  } finally {
    inFlight = null;
  }
}
