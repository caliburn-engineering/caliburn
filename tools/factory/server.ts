// Factory Floor — the server.
//
//   npm run factory                 → http://127.0.0.1:4600
//   HOST=0.0.0.0 npm run factory    → reachable over Tailscale from the phone
//
// Read-only over the trace db, plus a `gh issue list` the browser cannot make
// itself. The ingest loop re-parses changed .sandcastle/logs/*.log every few
// seconds, so a run in flight appears without anyone wiring anything.

import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { extname, join, normalize, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { openDb, type CommitRow, type EventRow, type PhaseRow, type RunRow } from "./db.js";
import { syncLogs } from "./ingest.js";
import { fetchIssues, fetchThread, issueByNumber, issueTitles } from "./issues.js";

const HERE = fileURLToPath(new URL(".", import.meta.url));
const PUBLIC_DIR = join(HERE, "public");
const REPO_ROOT = resolve(process.env.FACTORY_REPO ?? process.cwd());
const LOG_DIR = resolve(process.env.FACTORY_LOGS ?? join(REPO_ROOT, ".sandcastle/logs"));
const PORT = Number(process.env.PORT ?? 4600);
const HOST = process.env.HOST ?? "127.0.0.1";
const POLL_MS = Number(process.env.FACTORY_POLL_MS ?? 3000);

const db = openDb(process.env.FACTORY_DB ?? join(LOG_DIR, "trace.db"));

const MIME: Record<string, string> = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".svg": "image/svg+xml",
};

function json(body: unknown, status = 200) {
  const text = JSON.stringify(body);
  return { status, headers: { "content-type": "application/json; charset=utf-8", "cache-control": "no-store" }, text };
}

async function runSummaries(): Promise<Array<RunRow & { skills: string[]; commit_count: number; issue_title: string | null }>> {
  const runs = db.prepare(
    `SELECT * FROM runs ORDER BY (status = 'running') DESC, started_at DESC LIMIT 100`,
  ).all() as unknown as RunRow[];
  const skillStmt = db.prepare(
    "SELECT DISTINCT name FROM events WHERE run_id = ? AND type = 'skill' AND name IS NOT NULL",
  );
  const countStmt = db.prepare("SELECT COUNT(*) AS n FROM commits WHERE run_id = ?");
  // A run named afk-39 means nothing on its own; the ticket headline is what
  // tells you what the box is busy with. Empty when gh is unavailable.
  const titles = await issueTitles(db, REPO_ROOT);
  return runs.map((run) => ({
    ...run,
    skills: (skillStmt.all(run.run_id) as Array<{ name: string }>).map((r) => r.name),
    commit_count: (countStmt.get(run.run_id) as { n: number }).n,
    issue_title: run.issue === null ? null : titles.get(run.issue) ?? null,
  }));
}

async function runDetail(runId: string) {
  const run = db.prepare("SELECT * FROM runs WHERE run_id = ?").get(runId) as unknown as RunRow | undefined;
  if (!run) return null;
  return {
    run,
    issue: run.issue === null ? null : await issueByNumber(db, REPO_ROOT, run.issue),
    phases: db.prepare("SELECT * FROM phases WHERE run_id = ? ORDER BY seq").all(runId) as unknown as PhaseRow[],
    events: db.prepare("SELECT * FROM events WHERE run_id = ? ORDER BY seq").all(runId) as unknown as EventRow[],
    commits: db.prepare("SELECT * FROM commits WHERE run_id = ? ORDER BY committed_at").all(runId) as unknown as CommitRow[],
  };
}

async function handle(url: URL): Promise<{ status: number; headers: Record<string, string>; text?: string; body?: Buffer }> {
  if (url.pathname === "/api/health") {
    const runs = db.prepare("SELECT COUNT(*) AS n FROM runs").get() as { n: number };
    return json({ ok: true, runs: runs.n, logDir: LOG_DIR, repo: REPO_ROOT });
  }

  if (url.pathname === "/api/runs") return json({ runs: await runSummaries() });

  const detail = /^\/api\/runs\/(.+)$/.exec(url.pathname);
  if (detail) {
    const found = await runDetail(decodeURIComponent(detail[1]));
    return found ? json(found) : json({ error: "no such run" }, 404);
  }

  const thread = /^\/api\/issues\/(\d+)\/thread$/.exec(url.pathname);
  if (thread) {
    try {
      return json(await fetchThread(REPO_ROOT, Number(thread[1]), url.searchParams.get("force") === "1"));
    } catch (error) {
      return json({ number: Number(thread[1]), comments: [], error: (error as Error).message.split("\n")[0] });
    }
  }

  if (url.pathname === "/api/issues") {
    try {
      const issues = await fetchIssues(db, REPO_ROOT, url.searchParams.get("force") === "1");
      return json({ issues });
    } catch (error) {
      // gh not authed, offline, rate-limited — the runs half of the UI still works.
      return json({ issues: [], error: (error as Error).message.split("\n")[0] }, 200);
    }
  }

  // Static files. normalize() before join() so ../ cannot escape public/.
  const rel = url.pathname === "/" ? "index.html" : normalize(url.pathname).replace(/^(\.\.[/\\])+/, "").slice(1);
  const file = join(PUBLIC_DIR, rel);
  if (!file.startsWith(PUBLIC_DIR)) return { status: 403, headers: {}, text: "forbidden" };
  try {
    const body = await readFile(file);
    return { status: 200, headers: { "content-type": MIME[extname(file)] ?? "application/octet-stream" }, body };
  } catch {
    return { status: 404, headers: { "content-type": "text/plain" }, text: "not found" };
  }
}

const server = createServer((req, res) => {
  const url = new URL(req.url ?? "/", `http://${req.headers.host ?? "localhost"}`);
  handle(url)
    .then((result) => {
      res.writeHead(result.status, result.headers);
      res.end(result.body ?? result.text ?? "");
    })
    .catch((error: Error) => {
      res.writeHead(500, { "content-type": "application/json" });
      res.end(JSON.stringify({ error: error.message }));
    });
});

let ticking = false;
setInterval(() => {
  if (ticking) return; // a slow git lookup must not stack up behind itself
  ticking = true;
  try {
    syncLogs(db, LOG_DIR, REPO_ROOT);
  } catch (error) {
    console.error(`[factory] ingest: ${(error as Error).message}`);
  } finally {
    ticking = false;
  }
}, POLL_MS).unref();

try {
  const n = syncLogs(db, LOG_DIR, REPO_ROOT);
  console.log(`[factory] ingested ${n} log file(s) from ${LOG_DIR}`);
} catch (error) {
  console.error(`[factory] ingest failed: ${(error as Error).message}`);
}

server.listen(PORT, HOST, () => {
  console.log(`[factory] http://${HOST}:${PORT}  (repo: ${REPO_ROOT})`);
  if (HOST === "127.0.0.1") console.log("[factory] HOST=0.0.0.0 to reach it from the phone over Tailscale");
});
