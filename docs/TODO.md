# TODO / Task List for Wanderline
- Coding agentが一貫してタスクをこなせるように編集する領域。
- 同時に、ユーザーはこれを読んでCoding agentが今何をやっているかを確認することができる。
- ユーザーは最後のセクションに要望をメモしておくことで、coding agentに（抽象的・間違っていることもある）要求を共有できる。


## In Progress / Next Steps
- [x] Refactor run_test.py to support reward/loss function selection via config/CLI (l2, l2_white_penalty, etc.)
- [x] Add config/CLI option for white penalty alpha and other hyperparameters
- [x] Implement function selection logic (dict/factory) in run_test.py
- [x] Replace direct calls to compute_reward/l2_distance with selected function
- [x] Update outputs (distance curve, early stopping, summary) to use selected metric
- [x] Test both legacy and new reward/loss modes (unit/integration)
- [x] Update README and docs for new options and usage
- [x] Add transformer agent integration to run_test.py
- [ ] Implement actual transformer architecture (currently using MLP)
- [ ] Add expert trajectory generation for training data
- [ ] Implement supervised pre-training pipeline

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

## Transformer Agent Development Ideas
Current implementation uses greedy approach (1-step lookahead). Plan to implement Transformer-based agent for multi-step planning.

### Architecture Design
- [x] Design TransformerAgent class with sequence planning capability
- [x] Implement StateEncoder for multi-modal features (canvas, motif, position, history)
- [ ] Create TransformerModel for sequence prediction (currently using simple MLP)
- [ ] Add value/policy heads for RL integration

### State Representation
- [x] Enhance state encoding beyond pixel values
- [x] Add positional encoding for current drawing position
- [x] Include action history in state representation
- [x] Design canvas-motif feature fusion

### Training Strategy
- [ ] Generate expert trajectories from existing greedy agent
- [ ] Implement supervised pre-training on expert data
- [ ] Add reinforcement learning fine-tuning
- [ ] Consider Monte Carlo Tree Search integration

### Implementation Plan
- [x] Phase 1: Data preparation and baseline comparison
- [ ] Phase 2: Core transformer model implementation
- [ ] Phase 3: Training pipeline and evaluation
- [ ] Phase 4: Integration with existing run_test.py framework (partially done)

### Technical Considerations
- [ ] Determine optimal sequence length (10-20 steps)
- [ ] Design action space (discrete vs continuous angles)
- [ ] Plan reward shaping for multi-step optimization
- [ ] Consider computational efficiency for real-time drawing

---

*Update this file as tasks are added, started, or completed. Use checkboxes to track progress.*
