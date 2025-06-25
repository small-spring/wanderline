# Coding Rules

This document outlines the coding conventions and workflow guidelines for the Wanderline project.

- File names must be in English.
- Comments must be in English; do not use other languages in the code.
- Ensure tests pass and commit at appropriate times.
- Use clear and consistent file naming; avoid ad-hoc suffixes:
    - good: _v1
    - bad: _final
- After editing, run using `uv run`.
- Make small incremental changes and run tests frequently.
- After implementation, execute the code.
- Pay attention to code quality. Consider modularizing code if it exceeds 150 lines.
- **After tests pass, commit your changes.**
- You can use `scripts/test_and_commit.sh "your commit message"` to automatically run tests and commit if they pass.
    - Example: `scripts/test_and_commit.sh "feat: add white penalty to reward function and tests"`

Use `uv` for environment setup and package management:
- `uv run <python_file>`
- `uv add <python_packages>`