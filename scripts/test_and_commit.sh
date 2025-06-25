#!/usr/bin/env zsh

# Usage: scripts/test_and_commit.sh "your commit message"
msg="$1"
if [ -z "$msg" ]; then
  echo "Usage: $0 \"commit message\""
  exit 2
fi

uv run pytest -q
if [ $? -eq 0 ]; then
  git add .
  git commit -m "$msg"
  echo "Tests passed and changes committed."
else
  echo "Tests failed. Commit skipped." 1>&2
  exit 1
fi
