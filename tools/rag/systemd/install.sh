#!/usr/bin/env bash
# Install Caliburn RAG timer as a user systemd service.
#
# Usage: ./install.sh
#
# This copies the service and timer units to ~/.config/systemd/user/
# and enables the timer. To check status:
#   systemctl --user status caliburn-rag.timer
#   systemctl --user list-timers

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="${HOME}/.config/systemd/user"

mkdir -p "${SYSTEMD_DIR}"
cp "${SCRIPT_DIR}/caliburn-rag.service" "${SYSTEMD_DIR}/"
cp "${SCRIPT_DIR}/caliburn-rag.timer" "${SYSTEMD_DIR}/"

systemctl --user daemon-reload
systemctl --user enable caliburn-rag.timer
systemctl --user start caliburn-rag.timer

echo "✅ Caliburn RAG timer installed and started."
echo "   Next run: $(systemctl --user list-timers caliburn-rag.timer --no-pager | tail -2 | head -1)"
echo ""
echo "   Manual run:  systemctl --user start caliburn-rag.service"
echo "   Check logs:  journalctl --user -u caliburn-rag.service -f"
echo "   Disable:     systemctl --user disable caliburn-rag.timer"
