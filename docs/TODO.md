# TODO / Task List for Wanderline
- Coding agentが一貫してタスクをこなせるように編集する領域。
- 同時に、ユーザーはこれを読んでCoding agentが今何をやっているかを確認することができる。
- ユーザーは最後のセクションに要望をメモしておくことで、coding agentに（抽象的・間違っていることもある）要求を共有できる。


## In Progress / Next Steps
- [ ] Refactor run_test.py to support reward/loss function selection via config/CLI (l2, l2_white_penalty, etc.)
- [ ] Add config/CLI option for white penalty alpha and other hyperparameters
- [ ] Implement function selection logic (dict/factory) in run_test.py
- [ ] Replace direct calls to compute_reward/l2_distance with selected function
- [ ] Update outputs (distance curve, early stopping, summary) to use selected metric
- [ ] Test both legacy and new reward/loss modes (unit/integration)
- [ ] Update README and docs for new options and usage

## Completed
- [x] Modularize run_test.py (was 358 lines, now 46 lines with separate modules)
- [x] Fix `uv run python run_test.py` execution issue (line_width type error)
- [x] Add white penalty reward/loss functions to reward.py
- [x] Add/expand unit tests for all reward/loss functions (normal and error cases)
- [x] Make test_and_commit.sh accept commit message as argument
- [x] Update coding_rules.md and README.md for new workflow

## Ideas / Backlog
- [ ] Consider modularizing reward/strategy code if it grows
- [ ] Explore ticket/issue-based workflow if project scales up
- [ ] Add more advanced agent strategies (non-greedy, RL, etc.)
- [ ] Improve CLI help and documentation

## User Requests (coding agent will broke these down and move to other sections)
- linewidthの型についての確認 (resolved: kept as int, fixed config.json)

---

*Update this file as tasks are added, started, or completed. Use checkboxes to track progress.*
