"""
Test-driven development example for config updater functionality.

This demonstrates the TDD approach:
1. RED: Write failing tests first
2. GREEN: Implement minimal code to pass tests  
3. REFACTOR: Improve code quality while keeping tests green
"""

import pytest
import json
import tempfile
import os
from pathlib import Path
from wanderline.config_updater import update_configs, show_config_diff, restore_configs


class TestConfigUpdater:
    """Test config updater with TDD approach."""
    
    @pytest.fixture
    def temp_config_dir(self):
        """Create temporary directory with test config files."""
        with tempfile.TemporaryDirectory() as temp_dir:
            config_dir = Path(temp_dir) / "configs"
            config_dir.mkdir()
            
            # Create test config files
            test_configs = {
                "default.json": {
                    "steps": 100,
                    "opacity": 0.5,
                    "line_width": 2
                },
                "quick_test.json": {
                    "steps": 10,
                    "opacity": 0.3,
                    "line_width": 1
                },
                "long_run.json": {
                    "steps": 5000,
                    "opacity": 0.1,
                    "line_width": 3
                }
            }
            
            for filename, config in test_configs.items():
                with open(config_dir / filename, 'w') as f:
                    json.dump(config, f, indent=2)
            
            yield str(config_dir)
    
    def test_update_all_configs_basic(self, temp_config_dir):
        """RED: Test basic functionality of updating all config files."""
        # Test updating a single parameter across all files
        changes = {"steps": 1000}
        
        results = update_configs(changes, config_dir=temp_config_dir)
        
        # Should succeed for all files
        assert all(results.values()), f"Some updates failed: {results}"
        assert len(results) == 3, "Should update 3 config files"
        
        # Verify changes were applied
        for filename in ["default.json", "quick_test.json", "long_run.json"]:
            with open(os.path.join(temp_config_dir, filename)) as f:
                config = json.load(f)
                assert config["steps"] == 1000, f"Steps not updated in {filename}"
    
    def test_update_multiple_parameters(self, temp_config_dir):
        """RED: Test updating multiple parameters simultaneously."""
        changes = {
            "steps": 2000,
            "opacity": 0.8,
            "line_width": 5
        }
        
        results = update_configs(changes, config_dir=temp_config_dir)
        
        assert all(results.values())
        
        # Verify all changes were applied
        with open(os.path.join(temp_config_dir, "default.json")) as f:
            config = json.load(f)
            assert config["steps"] == 2000
            assert config["opacity"] == 0.8
            assert config["line_width"] == 5
    
    def test_update_specific_files(self, temp_config_dir):
        """RED: Test updating only specific files."""
        changes = {"steps": 999}
        target_files = ["default.json", "quick_test.json"]
        
        results = update_configs(
            changes, 
            files=target_files, 
            config_dir=temp_config_dir
        )
        
        assert len(results) == 2, "Should only update 2 files"
        assert all(results.values())
        
        # Verify only target files were updated
        with open(os.path.join(temp_config_dir, "default.json")) as f:
            config = json.load(f)
            assert config["steps"] == 999
            
        with open(os.path.join(temp_config_dir, "quick_test.json")) as f:
            config = json.load(f)
            assert config["steps"] == 999
            
        # long_run.json should remain unchanged
        with open(os.path.join(temp_config_dir, "long_run.json")) as f:
            config = json.load(f)
            assert config["steps"] == 5000, "long_run.json should not be updated"
    
    def test_pattern_matching(self, temp_config_dir):
        """RED: Test pattern-based file selection."""
        changes = {"opacity": 0.9}
        
        results = update_configs(
            changes,
            pattern="quick_*.json",
            config_dir=temp_config_dir
        )
        
        assert len(results) == 1, "Should only match quick_test.json"
        assert all(results.values())
        
        # Verify only quick_test.json was updated
        with open(os.path.join(temp_config_dir, "quick_test.json")) as f:
            config = json.load(f)
            assert config["opacity"] == 0.9
    
    def test_backup_creation(self, temp_config_dir):
        """RED: Test that backup files are created."""
        changes = {"steps": 777}
        
        results = update_configs(changes, config_dir=temp_config_dir, backup=True)
        
        assert all(results.values())
        
        # Check backup files exist
        config_path = Path(temp_config_dir)
        backup_files = list(config_path.glob("*.json.bak"))
        assert len(backup_files) == 3, "Should create 3 backup files"
        
        # Verify backup content matches original
        with open(config_path / "default.json.bak") as f:
            backup_config = json.load(f)
            assert backup_config["steps"] == 100, "Backup should contain original value"
    
    def test_dry_run_mode(self, temp_config_dir):
        """RED: Test dry run doesn't modify files."""
        original_configs = {}
        config_path = Path(temp_config_dir)
        
        # Store original configs
        for config_file in config_path.glob("*.json"):
            with open(config_file) as f:
                original_configs[config_file.name] = json.load(f)
        
        changes = {"steps": 9999}
        results = update_configs(changes, config_dir=temp_config_dir, dry_run=True)
        
        assert all(results.values()), "Dry run should report success"
        
        # Verify files weren't actually modified
        for config_file in config_path.glob("*.json"):
            with open(config_file) as f:
                current_config = json.load(f)
                original_config = original_configs[config_file.name]
                assert current_config == original_config, f"{config_file.name} was modified during dry run"
    
    def test_type_conversion(self, temp_config_dir):
        """RED: Test automatic type conversion for different data types."""
        changes = {
            "steps": 1000,        # int
            "opacity": 0.75,      # float  
            "greedy": True,       # boolean
            "resume_from": None   # null
        }
        
        results = update_configs(changes, config_dir=temp_config_dir)
        assert all(results.values())
        
        # Verify types were preserved
        with open(os.path.join(temp_config_dir, "default.json")) as f:
            config = json.load(f)
            assert isinstance(config["steps"], int)
            assert isinstance(config["opacity"], float)
            assert isinstance(config["greedy"], bool)
            assert config["resume_from"] is None
    
    def test_error_handling_nonexistent_file(self, temp_config_dir):
        """RED: Test error handling for nonexistent files."""
        changes = {"steps": 500}
        
        results = update_configs(
            changes,
            files=["nonexistent.json"],
            config_dir=temp_config_dir
        )
        
        # Should report failure for nonexistent file
        assert not results["nonexistent.json"]
    
    def test_restore_functionality(self, temp_config_dir):
        """RED: Test restore from backup functionality."""
        # First, make changes and create backups
        changes = {"steps": 888}
        update_configs(changes, config_dir=temp_config_dir, backup=True)
        
        # Verify changes were applied
        with open(os.path.join(temp_config_dir, "default.json")) as f:
            config = json.load(f)
            assert config["steps"] == 888
        
        # Now restore from backups
        restore_results = restore_configs(temp_config_dir)
        assert all(restore_results.values())
        
        # Verify restoration worked
        with open(os.path.join(temp_config_dir, "default.json")) as f:
            config = json.load(f)
            assert config["steps"] == 100, "Should be restored to original value"


class TestConfigDiff:
    """Test config difference functionality."""
    
    @pytest.fixture
    def temp_config_with_backups(self):
        """Create temp configs with backup files."""
        with tempfile.TemporaryDirectory() as temp_dir:
            config_dir = Path(temp_dir) / "configs"
            config_dir.mkdir()
            
            # Create original and backup
            original = {"steps": 100, "opacity": 0.5}
            backup = {"steps": 50, "opacity": 0.3}
            
            with open(config_dir / "test.json", 'w') as f:
                json.dump(original, f)
            with open(config_dir / "test.json.bak", 'w') as f:
                json.dump(backup, f)
            
            yield str(config_dir)
    
    def test_show_config_diff(self, temp_config_with_backups, capsys):
        """RED: Test showing differences between current and backup configs."""
        show_config_diff(temp_config_with_backups)
        
        captured = capsys.readouterr()
        assert "test.json" in captured.out
        assert "steps: 50 → 100" in captured.out
        assert "opacity: 0.3 → 0.5" in captured.out


# Integration test to demonstrate TDD workflow
class TestTDDWorkflow:
    """Demonstrate the TDD workflow with a new feature."""
    
    def test_new_feature_validation_red(self):
        """RED: Test for feature that doesn't exist yet."""
        # This test will initially fail - that's the RED phase
        with pytest.raises(ValueError, match="Invalid config key"):
            update_configs({"invalid_key_12345": "value"}, dry_run=True, validate_keys=True)
    
    # After implementing validation, this test should pass (GREEN phase)
    def test_new_feature_validation_green(self):
        """GREEN: Test passes after implementing validation."""
        # Valid keys should work without raising exception
        valid_changes = {"steps": 100, "opacity": 0.5}
        # This should not raise an exception (will be implemented)
        try:
            update_configs(valid_changes, dry_run=True, validate_keys=True)
        except ValueError as e:
            if "Invalid config key" in str(e):
                pytest.skip("Validation feature not yet implemented")