#!/usr/bin/env zsh

# テストを実行し、成功時にGitへコミットするスクリプト
uv run pytest -q
if [ $? -eq 0 ]; then
  git add .
  git commit -m "chore: tests passed"
  echo "Tests passed and changes committed."
else
  echo "Tests failed. Commit skipped." 1>&2
  exit 1
fi
