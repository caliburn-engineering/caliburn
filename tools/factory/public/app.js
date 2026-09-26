// Factory Floor — the client.
//
// Polling, not websockets: the server has no push and the data is a few hundred
// rows. Runs refresh every 3s, the issue board every 30s (a `gh` call costs
// more than a sqlite read), and nothing polls while the tab is hidden.

const RUNS_MS = 3000;
const ISSUES_MS = 30_000;
const THREAD_MS = 25_000;

const state = {
  tab: "runs",
  runs: [],
  selected: null,
  detail: null,
  issues: [],
  issueError: null,
  thread: null,
  updatedAt: null,
};

const $ = (id) => document.getElementById(id);

const esc = (s) =>
  String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

/** The agent writes markdown prose; only bold and code are worth honouring. */
const prose = (s) =>
  esc(s)
    .replace(/\*\*([^*]+)\*\*/g, "<b>$1</b>")
    .replace(/`([^`]+)`/g, "<code>$1</code>");

/**
 * Ticket bodies are fuller markdown — fenced blocks and headings carry meaning
 * you lose if they render as literal hashes and backticks. Fences are pulled out
 * first so their contents are never treated as inline markup.
 */
function markdownish(text) {
  const fences = [];
  const held = String(text ?? "").replace(/```[a-z]*\n?([\s\S]*?)```/gi, (_, code) => {
    fences.push(code.replace(/\n$/, ""));
    return `\u0000${fences.length - 1}\u0000`;
  });
  const rendered = prose(held)
    .replace(/^(#{1,6})\s+(.+)$/gm, "<b>$2</b>")
    .replace(/^(\s*)[-*]\s+/gm, "$1· ");
  return rendered.replace(/\u0000(\d+)\u0000/g, (_, i) => `<pre>${esc(fences[Number(i)])}</pre>`);
}

/** "4m ago" reads better than a timestamp for something that just happened. */
function ago(iso) {
  if (!iso) return "";
  const s = Math.round((Date.now() - new Date(iso).getTime()) / 1000);
  if (s < 90) return "just now";
  const m = Math.floor(s / 60);
  if (m < 60) return `${m}m ago`;
  const h = Math.floor(m / 60);
  if (h < 24) return `${h}h ago`;
  const d = Math.floor(h / 24);
  return d < 14 ? `${d}d ago` : new Date(iso).toLocaleDateString();
}

const squash = (text) => String(text ?? "").replace(/\s+/g, " ").trim().slice(0, 60);

function since(a, b) {
  if (!a) return null;
  const ms = (b ? new Date(b) : new Date()).getTime() - new Date(a).getTime();
  if (!Number.isFinite(ms) || ms < 0) return null;
  const s = Math.round(ms / 1000);
  if (s < 60) return `${s}s`;
  const m = Math.floor(s / 60);
  if (m < 60) return `${m}m ${s % 60}s`;
  return `${Math.floor(m / 60)}h ${m % 60}m`;
}

/**
 * Elapsed time, marked when it is only an upper bound. A run that is not the last
 * in its log file has no readable end, so its window stretches to the next run's
 * start — showing that as a flat number would be a confident lie.
 */
function elapsed(run) {
  const value = since(run.started_at, run.status === "running" ? null : run.ended_at);
  if (!value) return null;
  return run.end_bounded ? `≤ ${value}` : value;
}

const ms = (n) => (n === null || n === undefined ? "" : n >= 1000 ? `${(n / 1000).toFixed(1)}s` : `${n}ms`);

const clock = (iso) =>
  iso ? new Date(iso).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" }) : "";

// ---------- data ----------

async function get(path) {
  const res = await fetch(path, { headers: { accept: "application/json" } });
  if (!res.ok) throw new Error(`${res.status} ${path}`);
  return res.json();
}

async function refreshRuns() {
  const { runs } = await get("/api/runs");
  state.runs = runs;
  state.updatedAt = new Date();
  if (!state.selected && runs.length) state.selected = runs[0].run_id;
  if (state.selected) {
    const known = runs.some((r) => r.run_id === state.selected);
    if (!known) state.selected = runs.length ? runs[0].run_id : null;
  }
  if (state.selected) state.detail = await get(`/api/runs/${encodeURIComponent(state.selected)}`);
  await refreshThread().catch(() => { state.thread = null; });
  render();
}

/**
 * The ticket thread for whichever run is selected. Fetched separately from the
 * board so switching runs pulls one issue, not fifty — and refetched on a slow
 * beat so a reply you just posted on GitHub turns up without a page reload.
 */
async function refreshThread(force = false) {
  const number = state.detail?.run?.issue ?? null;
  if (number === null) { state.thread = null; return; }
  const held = state.thread;
  if (!force && held && held.number === number && Date.now() - held.at < THREAD_MS) return;
  const data = await get(`/api/issues/${number}/thread`);
  state.thread = { number, at: Date.now(), ...data };
}

async function refreshIssues() {
  const data = await get("/api/issues");
  state.issues = data.issues ?? [];
  state.issueError = data.error ?? null;
  render();
}

// ---------- runs ----------

function runCard(run) {
  const status = run.status ?? "unknown";
  const dur = elapsed(run);
  return `
    <button class="run-card ${status}" data-run="${esc(run.run_id)}"
            aria-current="${run.run_id === state.selected}">
      <span class="row">
        <span class="dot ${status}"></span>
        <span class="name">${esc(run.name ?? run.run_id)}</span>
        ${run.issue ? `<span class="chip issue">#${run.issue}</span>` : ""}
      </span>
      ${run.issue_title ? `<span class="headline">${esc(run.issue_title)}</span>` : ""}
      <span class="stats">
        <span>${clock(run.started_at)}</span>
        ${dur ? `<span>${dur}</span>` : ""}
        <span>${run.iterations}/${run.max_iterations ?? "?"} iter</span>
        <span>${run.tool_calls} tools</span>
        ${run.commit_count ? `<span>${run.commit_count} commit${run.commit_count > 1 ? "s" : ""}</span>` : ""}
      </span>
    </button>`;
}

function phaseRows(phases, events) {
  const longest = Math.max(1, ...phases.map((p) => p.duration_ms ?? 0));
  return phases
    .map((p) => {
      if (p.kind === "iteration") {
        const tools = events.filter((e) => e.iteration === p.iteration && e.type === "tool").length;
        return `<div class="phase iteration">
          <span class="label">${esc(p.name)}</span>
          <span class="track"></span>
          <span class="ms">${tools} tools</span>
        </div>`;
      }
      const width = ((p.duration_ms ?? 0) / longest) * 100;
      return `<div class="phase">
        <span class="label">${esc(p.name)}</span>
        <span class="track"><span class="bar" style="width:${width.toFixed(1)}%"></span></span>
        <span class="ms">${ms(p.duration_ms)}</span>
      </div>`;
    })
    .join("");
}

function timeline(events) {
  let iteration = null;
  const out = [];
  for (const e of events) {
    if (e.iteration !== iteration && e.iteration !== null) {
      iteration = e.iteration;
      out.push(`<div class="ev note"><span class="gutter">—</span><span class="body">iteration ${iteration}</span></div>`);
    }
    if (e.type === "text") {
      out.push(`<div class="ev text"><span class="gutter">says</span><span class="body">${prose(e.detail)}</span></div>`);
    } else if (e.type === "tool") {
      out.push(`<div class="ev tool"><span class="gutter">${esc(e.name)}</span><span class="body">${esc(e.detail)}</span></div>`);
    } else if (e.type === "report") {
      out.push(`<div class="ev report-ev"><span class="gutter">${esc(e.name)}</span><span class="body">${esc(e.detail)}</span></div>`);
    } else if (e.type === "note" && e.name === "failed") {
      out.push(`<div class="ev failed-ev"><span class="gutter">failed</span><span class="body">${esc(e.detail)}</span></div>`);
    } else if (e.type === "skill") {
      out.push(`<div class="ev skill"><span class="gutter">skill</span><span class="body">${esc(e.name)} · via ${esc(e.detail)}</span></div>`);
    } else {
      out.push(`<div class="ev note"><span class="gutter">${esc(e.name ?? "note")}</span><span class="body">${esc(e.detail)}</span></div>`);
    }
  }
  return out.join("");
}

/** What the run is working on, straight from the tracker. */
function ticketPanel(issue) {
  if (!issue) return "";
  return `
    <div class="panel">
      <h2>Ticket</h2>
      <div class="ticket-head">
        <a class="ticket-link" href="${esc(issue.url)}" target="_blank" rel="noreferrer">#${issue.number}</a>
        <span class="ticket-title">${esc(issue.title)}</span>
        <span class="chip ${issue.state === "CLOSED" ? "" : "issue"}">${issue.state === "CLOSED" ? "closed" : "open"}</span>
        ${issue.labels.map((l) => `<span class="chip">${esc(l)}</span>`).join("")}
        ${issue.wayfinder.map((w) => `<span class="chip way">${esc(w)}</span>`).join("")}
        ${issue.blockedBy.map((n) => `<span class="chip blocked">blocked by #${n}</span>`).join("")}
      </div>
      ${issue.body
        ? `<div class="clamp" title="click to expand">${markdownish(issue.body)}</div>`
        : `<p class="empty">no description on the ticket</p>`}
    </div>`;
}

/**
 * What the run concluded, before you merge or close anything.
 *
 * Three sources, most authoritative first: the comment it filed on the ticket
 * (prompt.md tells it to explain itself there, so this is where a question back
 * to a human lands), its closing prose, and — when the harness died rather than
 * the agent finishing — the failure itself, which matters more than either.
 */
function responsePanel(run, events) {
  const reports = events.filter((e) => e.type === "report");
  const failure = events.filter((e) => e.type === "note" && e.name === "failed").pop();
  if (!run.summary && !reports.length && !failure) return "";

  const live = run.status === "running";
  const title = failure ? "Run did not finish" : live ? "Latest from the agent" : "Conclusion";
  // The prose is worth a second look only when it says something the report did not.
  const proseWorthShowing = run.summary && (live || !reports.length || run.summary.length > 200);

  return `
    <div class="panel response ${failure ? "fail" : live ? "live" : ""}">
      <h2>${title}
        ${live ? `<span class="dot running"></span>` : ""}
        ${!live && !failure && run.completed ? `<span class="soft">· signalled COMPLETE</span>` : ""}
      </h2>
      ${failure ? `<p class="failline">${esc(failure.detail)}</p>` : ""}
      ${reports.map((r) => `
        <div class="report">
          <div class="report-head">${esc(r.name)}</div>
          <div class="response-body">${markdownish(r.detail)}</div>
        </div>`).join("")}
      ${proseWorthShowing ? `
        <div class="report">
          ${reports.length ? `<div class="report-head">closing words</div>` : ""}
          <div class="response-body">${markdownish(run.summary)}</div>
        </div>` : ""}
    </div>`;
}

/**
 * The live ticket thread. Two flags earn their place: which comment this run
 * filed (matched against the report in its own trace, since the agent posts under
 * your token and the author name cannot tell you), and which comments arrived
 * AFTER the run stopped — those are the ones nobody has answered yet.
 */
function threadPanel(run, events, thread) {
  if (!thread || thread.number !== run.issue) return "";
  if (thread.error) {
    return `<div class="panel"><h2>Ticket thread</h2><p class="error">gh: ${esc(thread.error)}</p></div>`;
  }
  const reports = events.filter((e) => e.type === "report").map((e) => squash(e.detail));
  const endedAt = run.ended_at ? new Date(run.ended_at).getTime() : null;

  const body = thread.comments.length
    ? thread.comments
        .map((c) => {
          const mine = reports.includes(squash(c.body));
          const after = endedAt !== null && new Date(c.createdAt).getTime() > endedAt + 60_000;
          return `<div class="comment-row ${mine ? "mine" : ""} ${after ? "after" : ""}">
            <div class="comment-head">
              <a href="${esc(c.url)}" target="_blank" rel="noreferrer">${esc(c.author)}</a>
              <span class="soft">${ago(c.createdAt)}</span>
              ${mine ? `<span class="chip">this run</span>` : ""}
              ${after && !mine ? `<span class="chip way">after the run</span>` : ""}
            </div>
            <div class="clamp">${markdownish(c.body)}</div>
          </div>`;
        })
        .join("")
    : `<p class="empty">no comments on the ticket yet</p>`;

  return `
    <div class="panel">
      <h2>Ticket thread
        <span class="soft">· ${thread.comments.length} comment${thread.comments.length === 1 ? "" : "s"}${
          thread.state === "CLOSED" ? ` · closed ${ago(thread.closedAt)}` : " · still open"
        }</span>
      </h2>
      ${body}
    </div>`;
}

function renderDetail() {
  const host = $("run-detail");
  if (!state.detail) {
    host.innerHTML = `<div class="panel"><p class="empty">No runs yet. Launch one with <code>work caliburn afk</code> — it appears here within a few seconds.</p></div>`;
    return;
  }
  const { run, phases, events, commits, issue } = state.detail;
  const summary = state.runs.find((r) => r.run_id === run.run_id);
  const skills = summary?.skills ?? [];
  const dur = elapsed(run);

  host.innerHTML = `
    <div class="panel">
      <div class="detail-head">
        <span class="dot ${run.status}"></span>
        <span class="title">${esc(run.name ?? run.run_id)}</span>
        ${run.issue ? `<span class="chip issue">issue #${run.issue}</span>` : ""}
        ${issue ? `<span class="headline">${esc(issue.title)}</span>` : ""}
        ${run.completed ? `<span class="chip">COMPLETE signal</span>` : ""}
      </div>
      <div class="kv">
        <span>status <b>${esc(run.status)}</b></span>
        <span>branch <b>${esc(run.branch ?? "—")}</b></span>
        <span>harness <b>${esc(run.harness ?? "—")}</b></span>
        <span>sandbox <b>${esc(run.sandbox ?? "—")}</b></span>
        <span>iterations <b>${run.iterations}/${run.max_iterations ?? "?"}</b></span>
        <span>tools <b>${run.tool_calls}</b></span>
        ${dur ? `<span${run.end_bounded ? ' title="upper bound — the log carries no end timestamp, so this run\'s window runs to the next run\'s start"' : ""}>elapsed <b>${dur}</b></span>` : ""}
        <span>started <b>${clock(run.started_at)}</b></span>
      </div>
      ${skills.length ? `<div class="kv" style="margin-top:10px">${skills.map((s) => `<span class="chip skill">${esc(s)}</span>`).join("")}</div>` : ""}
    </div>

    ${ticketPanel(issue)}
    ${responsePanel(run, events)}
    ${threadPanel(run, events, state.thread)}

    <div class="panel">
      <h2>Phases</h2>
      <div class="phases">${phaseRows(phases, events)}</div>
    </div>

    ${commits.length ? `<div class="panel">
      <h2>Commits</h2>
      ${commits.map((c) => `<div class="commit"><span class="sha">${esc(c.sha.slice(0, 7))}</span><span>${esc(c.subject)}</span></div>`).join("")}
    </div>` : ""}

    <div class="panel">
      <h2>Trace <span style="color:var(--dim);text-transform:none;letter-spacing:0">· ${events.length} events</span></h2>
      <div class="timeline">${timeline(events)}</div>
    </div>`;
}

// ---------- issues ----------

const COLUMNS = [
  ["in-flight", "In flight"],
  ["ready-for-agent", "Ready for agent"],
  ["needs-triage", "Needs triage"],
  ["needs-info", "Needs info"],
  ["ready-for-human", "Ready for human"],
  ["unlabelled", "No triage label"],
  ["done", "Done"],
  ["wontfix", "Won't fix"],
];

function issueCard(issue) {
  return `
    <a class="issue ${issue.activeRun ? "working" : ""}" href="${esc(issue.url)}" target="_blank" rel="noreferrer">
      <span class="num">#${issue.number}</span>
      <span class="title">${esc(issue.title)}</span>
      <span class="tags">
        ${issue.activeRun ? `<span class="chip issue">running</span>` : ""}
        ${issue.blockedBy.map((n) => `<span class="chip blocked">blocked by #${n}</span>`).join("")}
        ${issue.wayfinder.map((w) => `<span class="chip way">${esc(w)}</span>`).join("")}
        ${issue.commits.map((c) => `<span class="chip">${esc(c.sha)}</span>`).join("")}
      </span>
    </a>`;
}

function renderBoard() {
  $("issue-error").innerHTML = state.issueError
    ? `<p class="error">gh: ${esc(state.issueError)}</p>`
    : "";
  const board = $("board");
  const groups = new Map(COLUMNS.map(([key]) => [key, []]));
  for (const issue of state.issues) groups.get(issue.column)?.push(issue);

  board.innerHTML = COLUMNS.filter(([key]) => (groups.get(key) ?? []).length || key === "ready-for-agent" || key === "in-flight")
    .map(([key, label]) => {
      const items = groups.get(key) ?? [];
      items.sort((a, b) => (b.updatedAt > a.updatedAt ? 1 : -1));
      const shown = key === "done" || key === "wontfix" ? items.slice(0, 12) : items;
      return `<div class="column ${key}">
        <h3>${label}<span class="count">${items.length}</span></h3>
        ${shown.map(issueCard).join("") || `<p class="empty">nothing here</p>`}
        ${items.length > shown.length ? `<p class="empty">+ ${items.length - shown.length} older</p>` : ""}
      </div>`;
    })
    .join("");
}

// ---------- shell ----------

function render() {
  const running = state.runs.filter((r) => r.status === "running").length;
  $("live").innerHTML = running
    ? `<span class="dot running"></span> ${running} running`
    : `<span class="dot"></span> idle`;
  $("updated").textContent = state.updatedAt ? `updated ${clock(state.updatedAt.toISOString())}` : "";

  $("run-list").innerHTML = state.runs.map(runCard).join("") || `<p class="empty">no runs yet</p>`;
  renderDetail();
  renderBoard();
}

function selectTab(tab) {
  state.tab = tab;
  for (const name of ["runs", "issues"]) {
    $(`tab-${name}`).setAttribute("aria-selected", String(name === tab));
    $(`view-${name}`).hidden = name !== tab;
  }
  if (tab === "issues") refreshIssues().catch(() => {});
}

document.addEventListener("click", (event) => {
  const card = event.target.closest?.(".run-card");
  if (card) {
    state.selected = card.dataset.run;
    state.detail = null;
    state.thread = null;
    refreshRuns().catch(() => {});
    return;
  }
  const body = event.target.closest?.(".ev.tool .body, .ev.skill .body, .ev.report-ev .body, .clamp");
  if (body) body.classList.toggle("open");
});

$("tab-runs").addEventListener("click", () => selectTab("runs"));
$("tab-issues").addEventListener("click", () => selectTab("issues"));

const tick = (fn, every) => {
  const beat = () => {
    if (!document.hidden) fn().catch((error) => console.warn(error));
  };
  beat();
  setInterval(beat, every);
};

tick(refreshRuns, RUNS_MS);
tick(refreshIssues, ISSUES_MS);
// Elapsed time on a live run should keep moving between polls.
setInterval(() => { if (!document.hidden && state.runs.some((r) => r.status === "running")) render(); }, 1000);
