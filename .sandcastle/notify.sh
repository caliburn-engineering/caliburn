#!/usr/bin/env bash
# Sandcastle AFK notifier — host-side bridge for unattended agent runs.
#
# Sandcastle runs the agent INSIDE a container, so the global Claude Code
# Stop/Notification hooks (~/.claude/hooks/*.sh) never fire for AFK runs and the
# statusline shows nothing. main.ts calls this from logging.onAgentStreamEvent
# and at run boundaries so a phone still learns what happened.
#
# Reuses Mobile-Workflow's routing contract:
#   /tmp/claude-tts-override  — "off" disables speech (notify-send still fires)
#   /tmp/claude-tts-route     — "phone" | "desktop"
#
# Usage: notify.sh "<title>" "<message>"

set -u

title="${1:-Sandcastle}"
message="${2:-}"

TOGGLE_FILE="/tmp/claude-tts-override"
ROUTE_FILE="/tmp/claude-tts-route"

# --- Desktop notification (always, rate-limit-free: run boundaries are rare) ---
if command -v notify-send >/dev/null 2>&1; then
    notify-send "$title" "$message" 2>/dev/null || true
fi

# --- Speech, only when TTS is toggled on ---
[[ -f "$TOGGLE_FILE" ]] || exit 0
[[ "$(cat "$TOGGLE_FILE")" == "off" ]] && exit 0

route="desktop"
[[ -f "$ROUTE_FILE" ]] && route=$(cat "$ROUTE_FILE")

speech="$title. $message"

if [[ "$route" == "phone" ]]; then
    tmp=$(mktemp)
    echo "$speech" > "$tmp"
    scp -o ConnectTimeout=3 -q "$tmp" phone-termux:~/sandcastle-tts.txt 2>/dev/null
    rm -f "$tmp"
    nohup ssh -o ConnectTimeout=3 phone-termux \
        "termux-tts-speak < ~/sandcastle-tts.txt" >/dev/null 2>&1 &
else
    command -v espeak-ng >/dev/null 2>&1 && espeak-ng "$speech" >/dev/null 2>&1 &
fi

exit 0
