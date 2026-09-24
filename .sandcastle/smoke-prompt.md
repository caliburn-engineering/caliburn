# Task

You are verifying that this sandbox is correctly provisioned for Caliburn. Do
not modify, create, or commit any files. Run checks and report.

## Checks

Run each command and note the result:

1. `pi --version` — must be **0.84.x**. If it reports 0.73.x, the Dockerfile is
   installing the old `@mariozechner/pi-coding-agent` package instead of
   `@earendil-works/pi-coding-agent`.
2. `claude --version` — the Claude Code CLI should also be present.
3. `ls ~/.claude/skills/` — must list Matt Pocock's engineering skills
   (`wayfinder`, `triage`, `to-tickets`, `implement`, `code-review`, …). An
   empty or missing directory means the host mount did not land.
4. `ls .pi/skills/` — must list Caliburn's own engineering skills
   (`design-controller`, `design-observer`, `validate-implementation`, …).
5. `cmake --version` and `g++ --version` — both must be present.
6. `git log --oneline -3` — confirm the repo history is visible.
7. `gh auth status` — confirm the GitHub CLI can authenticate with GH_TOKEN.
8. `ls knowledge/ reference/` — confirm the knowledge and reference trees are
   present in the worktree.
9. `ls projects/ 2>/dev/null || echo "projects/ absent (expected)"` — `projects/`
   is gitignored, so it is EXPECTED to be missing from a worktree-based run.

## Report

Output your findings inside `<report>` tags — a line per check, each marked PASS
or FAIL, then a one-line verdict. Example shape:

<report>
1. pi --version: PASS (0.84.4)
2. claude --version: FAIL (command not found)
...
VERDICT: not ready — Claude Code CLI missing from image
</report>
