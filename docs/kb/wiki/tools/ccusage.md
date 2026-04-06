# ccusage

CLI tool for analyzing Claude Code token usage and costs from local session history.
Reads JSONL files in `~/.claude/projects/` — no config needed.

**Repo:** https://github.com/ryoppippi/ccusage

## Commands

```bash
npx ccusage@latest          # daily report (default)
npx ccusage@latest monthly  # monthly aggregated
npx ccusage@latest session  # per-conversation breakdown
npx ccusage@latest blocks   # usage in 5-hour billing windows
```

## Installation

No install required — `npx` runs it directly. Requires Node.js (installed via nvm).

```bash
# If Node is not installed:
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
source ~/.bashrc
nvm install --lts
```

## Notes

