# TODO / Task List for Wanderline
- Coding agentが一貫してタス## User Requests (coding agent will broke these down and move to other sections)
- [x] 今更だけど、引数のratioとかalphaとかわかりにくいので改善したい。→ COMPLETED: absolute_penalty/relative_penalty に変更完了
- [x] 複数ステップ（指定できると良い）に対してgreedyできるように拡張したいかも。現在のバージョンを、step深さ（変数名は適切に考える必要がありそう）=1である時の挙動にして、増やせるようにしたい。→ Ideas/Backlog に移動

## Parameter Naming Discussion
Current white penalty parameters:
- `white_penalty_alpha` - 絶対ペナルティ強度（alphaは透明度と混同しやすい）
- `white_penalty_alpha_ratio` - 相対ペナルティ強度（長すぎる）

Proposed alternatives:
1. **Option A (Simple)**: `white_penalty` (float) - 相対ペナルティのみ、設定簡素化
2. **Option B (Explicit)**: `white_penalty_strength`, `white_penalty_ratio`
3. **Option C (Clear)**: `white_penalty_absolute`, `white_penalty_relative`

Arguments for Option A:
- 設定がシンプル（ユーザーの指摘通り、相対ペナルティが主に使われる）
- 絶対ペナルティが必要な場合は、相対ペナルティで小さい値を使用することで代替可能
- スケーリングの難しさを回避なせるように編集する領域。
- 同時に、ユーザーはこれを読んでCoding agentが今何をやっているかを確認することができる。
- ユーザーは最後のセクションに要望をメモしておくことで、coding agentに（抽象的・間違っていることもある）要求を共有できる。


## Documentation Update Checklist
When making changes to the codebase, update the following documents as needed:

### Code Changes → Documentation Updates
- **Reward/Loss Functions** (`wanderline/reward.py`) → `docs/config_guide.md`, `docs/README.md`
- **CLI Arguments** (`wanderline/config_manager.py`) → `docs/config_guide.md`, CLI help text
- **Config Parameters** (`config.sample.json`) → `docs/config_guide.md`
- **Agent Types** (`wanderline/agent.py`, `wanderline/drawing_engine.py`) → `docs/README.md`, `docs/config_guide.md`
- **New Features** → `docs/README.md`, `docs/specification.md`
- **API Changes** → All relevant docs, update examples

### Recently Updated Documents
- [x] `configs/` structure - Organized config files with clear naming
- [x] `docs/config_guide.md` - Updated for new config structure and simplified white penalty
- [x] `docs/README.md` - Reorganized documentation structure
- [ ] `docs/README.md` - Need to update with current white penalty usage examples

## In Progress / Next Steps
- [x] Improve argument naming for white penalty parameters (alpha -> absolute_penalty, alpha_ratio -> relative_penalty) - COMPLETED: Changed to absolute_penalty/relative_penalty
- [x] Clean up white penalty argument naming to avoid confusion with alpha (transparency) - COMPLETED: Simplified to single `white_penalty` parameter
- [x] Consider simplifying config by removing absolute penalty option (keep only relative penalty) - COMPLETED: Only scale-invariant penalty remains
- [x] Organize config files into configs/ folder with proper naming - COMPLETED: Moved to configs/ with descriptive names
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
- [x] Clean up config files and debug scripts
- [x] Remove duplicate run_test_v2.py file
- [x] Delete obsolete sample.json and debug files
- [x] Organize docs/ folder structure

## Ideas / Backlog
- [ ] Implement multi-step greedy search with configurable lookahead depth (moved from In Progress)
- [ ] Update CLI/config system to support lookahead_depth parameter
- [ ] Add unit tests for multi-step greedy agent
- [ ] Consider modularizing reward/strategy code if it grows
- [ ] Explore ticket/issue-based workflow if project scales up
- [ ] Add more advanced agent strategies (non-greedy, RL, etc.)
- [ ] Improve CLI help and documentation

## User Requests (coding agent will broke these down and move to other sections)
- [x] 今更だけど、引数のratioとかalphaとかわかりにくいので改善したい。→ COMPLETED: absolute_penalty/relative_penalty に変更完了
- [x] 複数ステップ（指定できると良い）に対してgreedyできるように拡張したいかも。現在のバージョンを、step深さ（変数名は適切に考える必要がありそう）=1である時の挙動にして、増やせるようにしたい。→ In Progress に移動

## Transformer Agent Development Ideas -　（一旦、完全に後回し）
Current implementation uses greedy approach (1-step lookahead). Plan to implement Transformer-based agent for multi-step planning.
これ、まずは正解軌道を用意しないと駄目ということに気がついたので後回し！

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
