# TODO / Task List for Wanderline

## In Progress / Next Steps
- [ ] Refactor run_test.py to support reward/loss function selection via config/CLI (l2, l2_white_penalty, etc.)
- [ ] Add config/CLI option for white penalty alpha and other hyperparameters
- [ ] Implement function selection logic (dict/factory) in run_test.py
- [ ] Replace direct calls to compute_reward/l2_distance with selected function
- [ ] Update outputs (distance curve, early stopping, summary) to use selected metric
- [ ] Test both legacy and new reward/loss modes (unit/integration)
- [ ] Update README and docs for new options and usage

## Completed
- [x] Add white penalty reward/loss functions to reward.py
- [x] Add/expand unit tests for all reward/loss functions (normal and error cases)
- [x] Make test_and_commit.sh accept commit message as argument
- [x] Update coding_rules.md and README.md for new workflow

## Ideas / Backlog
- [ ] Consider modularizing reward/strategy code if it grows
- [ ] Explore ticket/issue-based workflow if project scales up
- [ ] Add more advanced agent strategies (non-greedy, RL, etc.)
- [ ] Improve CLI help and documentation

## User Requests (to be broken down and moved to other sections)
- `uv run python run_test.py` だけで動かないのでそれを直したい。
- run_test.pyが長すぎる
- linewidthはintである必要はない気がする。linewidthを最終的に使っているところを特定し、確認したい

---

*Update this file as tasks are added, started, or completed. Use checkboxes to track progress.*
