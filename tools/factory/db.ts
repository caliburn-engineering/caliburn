// Factory Floor — the trace store.
//
// One SQLite file under .sandcastle/logs/ (already gitignored). The schema is
// deliberately close to IndyDevDan's sssf.db — sessions/phases/events — so his
// visualizer could read ours and vice versa, but the columns say Sandcastle
// things: branch, iteration, completion signal, skills.
//
// Everything here is synchronous on purpose. The writer is a log parser, not a
// hot path, and DatabaseSync keeps the code free of await noise.

import { DatabaseSync } from "node:sqlite";
import { mkdirSync } from "node:fs";
import { dirname, resolve } from "node:path";

export const DEFAULT_DB = ".sandcastle/logs/trace.db";

/**
 * A run is one `sandcastle run()` call. `name` is what main.ts passed (e.g.
 * "afk-39"), which is also the log basename — that is how a log file and a run
 * find each other on re-import.
 */
const SCHEMA = `
CREATE TABLE IF NOT EXISTS runs (
  run_id        TEXT PRIMARY KEY,
  name          TEXT,
  issue         INTEGER,            -- the issue this run worked, when known
  harness       TEXT,               -- claude | pi | unknown
  sandbox       TEXT,               -- docker | podman | none
  branch        TEXT,
  status        TEXT,               -- running | complete | stopped | failed | unknown
  max_iterations INTEGER,
  iterations    INTEGER DEFAULT 0,
  completed     INTEGER DEFAULT 0,  -- agent emitted <promise>COMPLETE</promise>
  tool_calls    INTEGER DEFAULT 0,
  log_path      TEXT,
  log_size      INTEGER DEFAULT 0,  -- re-import trigger: bytes parsed last time
  summary       TEXT,               -- the agent's closing words
  started_at    TEXT,
  ended_at      TEXT,
  end_bounded   INTEGER DEFAULT 0   -- 1 = ended_at is an upper bound, not a reading
);
CREATE TABLE IF NOT EXISTS phases (
  phase_id      TEXT PRIMARY KEY,
  run_id        TEXT,
  seq           INTEGER,
  iteration     INTEGER,            -- the iteration a setup step belongs to
  name          TEXT,               -- "Copying to worktree" | "Iteration 1"
  kind          TEXT,               -- setup | iteration | wrapup
  status        TEXT,               -- ok | running | fail
  duration_ms   INTEGER,            -- Sandcastle prints these for setup phases
  detail        TEXT
);
CREATE TABLE IF NOT EXISTS events (
  event_id      INTEGER PRIMARY KEY AUTOINCREMENT,
  run_id        TEXT,
  phase_id      TEXT,
  iteration     INTEGER,
  seq           INTEGER,            -- order within the run; the log has no timestamps
  type          TEXT,               -- text | tool | skill | note
  name          TEXT,               -- tool name, or skill name
  detail        TEXT
);
CREATE TABLE IF NOT EXISTS commits (
  run_id        TEXT,
  sha           TEXT,
  subject       TEXT,
  issue         INTEGER,
  committed_at  TEXT,
  PRIMARY KEY (run_id, sha)
);
CREATE TABLE IF NOT EXISTS log_files (
  path          TEXT PRIMARY KEY,   -- every log seen, including ones holding no run
  size          INTEGER,
  mtime_ms      INTEGER,
  runs          INTEGER DEFAULT 0   -- 0 is a legitimate answer: console logs hold none
);
CREATE INDEX IF NOT EXISTS events_run ON events(run_id, seq);
CREATE INDEX IF NOT EXISTS phases_run ON phases(run_id, seq);
`;

export type Db = DatabaseSync;

/**
 * WAL matters here: five concurrent AFK runs mean the server polls this file
 * while the importer writes it. WAL lets readers through without blocking.
 */
export function openDb(path?: string): Db {
  const file = resolve(path ?? process.env.FACTORY_DB ?? DEFAULT_DB);
  mkdirSync(dirname(file), { recursive: true });
  const db = new DatabaseSync(file);
  db.exec("PRAGMA journal_mode = WAL");
  db.exec("PRAGMA busy_timeout = 5000");
  db.exec(SCHEMA);
  return db;
}

export interface RunRow {
  run_id: string;
  name: string | null;
  issue: number | null;
  harness: string | null;
  sandbox: string | null;
  branch: string | null;
  status: string | null;
  max_iterations: number | null;
  iterations: number;
  completed: number;
  tool_calls: number;
  log_path: string | null;
  log_size: number;
  summary: string | null;
  started_at: string | null;
  ended_at: string | null;
  end_bounded: number;
}

export interface PhaseRow {
  phase_id: string;
  run_id: string;
  seq: number;
  iteration: number | null;
  name: string;
  kind: string;
  status: string;
  duration_ms: number | null;
  detail: string | null;
}

export interface EventRow {
  event_id: number;
  run_id: string;
  phase_id: string | null;
  iteration: number | null;
  seq: number;
  type: string;
  name: string | null;
  detail: string | null;
}

export interface CommitRow {
  run_id: string;
  sha: string;
  subject: string | null;
  issue: number | null;
  committed_at: string | null;
}
