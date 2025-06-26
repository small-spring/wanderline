# Development Guidelines for Wanderline

This document outlines the comprehensive development guidelines, coding conventions, and test-driven development practices for the Wanderline project.

## 1. Coding Conventions

### Language and Documentation
- **File names must be in English** - No Japanese characters in filenames
- **Comments must be in English** - Do not use other languages in code comments
- **Variable and function names** - Use clear, descriptive English names
- **Documentation** - Update docs/ files appropriately for future reference

### Code Quality Standards
- **Modularization** - Consider splitting code if it exceeds 150 lines
- **Clean code** - Remove unused functions and variables
- **Design decisions** - Consider which code variants to keep vs remove
- **File naming** - Use clear, consistent naming conventions:
  - ✅ Good: `config_v1.json`, `agent_vectorized.py`
  - ❌ Bad: `config_final.json`, `agent_new.py`

### Code Organization
- Keep functions focused and single-purpose
- Use meaningful variable names that explain intent
- Group related functionality in modules
- Remove deprecated functions after confirming they're no longer needed

## 2. Development Workflow

### Environment and Tools
```bash
# Use uv for all Python operations
uv run python script.py          # Execute Python files
uv run pytest                    # Run tests
uv add package_name              # Add dependencies
```

### Development Process
1. **Make small incremental changes** - Don't try to implement everything at once
2. **Run tests frequently** - After each logical change
3. **Execute code after implementation** - Verify it works in practice
4. **Test before commit** - Ensure all tests pass before committing

### Automated Testing and Commits
```bash
# Automatic test-and-commit workflow
scripts/test_and_commit.sh "feat: add white penalty to reward function and tests"
scripts/test_and_commit.sh "fix: resolve canvas rendering bug"
scripts/test_and_commit.sh "refactor: improve agent vectorization performance"
```

## 3. Test-Driven Development (TDD)

### TDD Cycle Overview
Follow the **RED-GREEN-REFACTOR** cycle:

1. **🔴 RED**: Write a failing test first
2. **🟢 GREEN**: Write minimal code to make the test pass  
3. **🔄 REFACTOR**: Improve code quality while keeping tests green

### TDD Workflow Example

#### Step 1: RED - Write Failing Test
```python
def test_config_validation():
    """Test that invalid config keys are rejected."""
    with pytest.raises(ValueError, match="Invalid config key"):
        update_configs({"invalid_key": "value"}, validate_keys=True)
```

#### Step 2: GREEN - Implement Minimal Solution
```python
def _validate_config_keys(changes, config_dir):
    """Validate that config keys exist."""
    # Minimal implementation to pass test
    if "invalid_key" in changes:
        raise ValueError("Invalid config key: invalid_key")
```

#### Step 3: REFACTOR - Improve Implementation
```python
def _validate_config_keys(changes, config_dir):
    """Validate that config keys exist in at least one config file."""
    config_path = Path(config_dir)
    valid_keys = set()
    
    # Collect all keys from existing config files
    for config_file in config_path.glob("*.json"):
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
                valid_keys.update(config.keys())
        except Exception:
            continue
    
    # Check if any change keys are invalid
    invalid_keys = set(changes.keys()) - valid_keys
    if invalid_keys:
        raise ValueError(f"Invalid config key(s): {', '.join(invalid_keys)}")
```

### Test Organization

#### Test File Structure
```
tests/
├── test_canvas.py           # Canvas operations
├── test_agent.py            # Agent algorithms  
├── test_config_updater.py   # Configuration management
├── test_reward.py           # Reward functions
└── test_integration.py      # End-to-end workflows
```

#### Test Categories

**Unit Tests** - Test individual functions:
```python
def test_apply_stroke_horizontal():
    """Test stroke application in horizontal direction."""
    canvas = np.ones((100, 100, 3), dtype=np.uint8) * 255
    result = apply_stroke(canvas, angle=0.0)
    assert np.all(result[center_y, center_x] == [0, 0, 0])
```

**Integration Tests** - Test module interactions:
```python
def test_config_loading_with_agent():
    """Test complete workflow from config to agent."""
    args = load_config_and_parse_args()
    agent = create_agent(args)
    assert agent is not None
```

**Property Tests** - Test with various inputs:
```python
@pytest.mark.parametrize("angle", [0, np.pi/4, np.pi/2, np.pi])
def test_stroke_angles(angle):
    """Test stroke application at various angles."""
    result = apply_stroke(canvas, angle=angle)
    assert result is not None
```

### Running Tests

#### Daily Development Workflow
```bash
# Before starting work
uv run pytest

# After making changes  
uv run pytest tests/test_module_you_changed.py

# Run specific test
uv run pytest tests/test_canvas.py::test_apply_stroke_horizontal -v

# Before committing
uv run pytest
```

#### Test Coverage Goals
- **Core functions** (canvas, agent, reward): 100% coverage
- **Configuration**: 90% coverage  
- **Utilities**: 80% coverage
- **Integration**: Key workflows covered

### Best Practices

#### Test Naming
```python
def test_function_name_scenario():
    """Test that function_name behaves correctly when scenario occurs."""
    # Arrange - Set up test data
    # Act - Execute the function
    # Assert - Verify the results
```

#### Fixtures for Test Data
```python
@pytest.fixture
def sample_canvas():
    """Provide standard test canvas."""
    return np.ones((100, 100, 3), dtype=np.uint8) * 255

@pytest.fixture
def temp_config_dir():
    """Provide temporary config directory."""
    with tempfile.TemporaryDirectory() as temp_dir:
        # Setup test configs
        yield temp_dir
```

#### Effective Assertions
```python
# Good: Specific assertions
assert config["steps"] == 1000
assert len(results) == 3

# Better: Descriptive error messages  
assert config["steps"] == 1000, f"Expected steps=1000, got {config['steps']}"

# Best: Use pytest helpers
assert result == pytest.approx(expected, rel=1e-5)
```

## 4. Adding New Features

### TDD Process for New Features

1. **Write the test first** (RED):
   ```python
   def test_new_stroke_validation():
       with pytest.raises(ValueError):
           apply_stroke(canvas, angle=-1)  # Invalid angle
   ```

2. **Run the test** - it should fail:
   ```bash
   uv run pytest tests/test_canvas.py::test_new_stroke_validation -v
   ```

3. **Write minimal implementation** (GREEN):
   ```python
   def apply_stroke(canvas, angle):
       if angle < 0:
           raise ValueError("Angle must be non-negative")
       # Rest of implementation
   ```

4. **Run test again** - it should pass

5. **Refactor** while keeping tests green:
   ```python  
   def apply_stroke(canvas, angle):
       if not (0 <= angle < 2 * np.pi):
           raise ValueError(f"Angle must be in [0, 2π), got {angle}")
       # Improved validation with better range checking
   ```

### Configuration Management

Use the config updater for batch modifications:

```bash
# Update all config files
uv run python scripts/update_configs.py --set steps=1000

# Update specific files
uv run python scripts/update_configs.py --set opacity=0.5 --files default.json long_run.json

# Update with pattern matching
uv run python scripts/update_configs.py --set duration=10.0 --pattern "quick_*.json"

# Preview changes without applying
uv run python scripts/update_configs.py --set steps=500 --dry-run

# Show differences from backups
uv run python scripts/update_configs.py --show-diff

# Restore from backups
uv run python scripts/update_configs.py --restore
```

## 5. Documentation Maintenance

### Regular Updates
- Update `docs/` files when adding new features
- Remove obsolete documentation when deprecating features
- Keep `CLAUDE.md` current with new commands and workflows
- Propose removal of outdated content in discussions

### Version Control
- Commit documentation changes with code changes
- Use descriptive commit messages for documentation updates
- Keep documentation in sync with implementation

## 6. Performance and Quality

### Memory Management
- Use memory-efficient mode for long runs (>1000 steps)
- Monitor memory usage during development
- Profile performance-critical code paths

### Code Review
- Check for unused imports and functions
- Verify test coverage for new features
- Ensure error handling is appropriate
- Validate that documentation is updated

---

## Quick Reference

### Common Commands
```bash
# Run all tests
uv run pytest

# Run specific test file  
uv run pytest tests/test_canvas.py

# Run with verbose output
uv run pytest tests/test_canvas.py -v

# Test and commit if passing
scripts/test_and_commit.sh "your commit message"

# Update config files
uv run python scripts/update_configs.py --set key=value
```

### TDD Checklist
- [ ] Write failing test first (RED)
- [ ] Implement minimal solution (GREEN)  
- [ ] Refactor for quality (REFACTOR)
- [ ] All tests pass
- [ ] Documentation updated
- [ ] Commit changes

This comprehensive guide ensures consistent, high-quality development practices across the Wanderline project.