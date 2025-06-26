"""
Test-Driven Development for Practical Config Class
=================================================

RED-GREEN-REFACTOR cycle implementation:
1. RED: Write failing tests first
2. GREEN: Make tests pass with minimal code
3. REFACTOR: Improve code while keeping tests green
"""

import pytest
from argparse import Namespace
from wanderline.practical_config import WanderlineConfig, create_config
import tempfile
import json
import os


class TestWanderlineConfigBasics:
    """Test basic configuration functionality."""
    
    def test_default_config_creation(self):
        """Should create config with sensible defaults."""
        config = WanderlineConfig()
        
        assert config.motif_path is None
        assert config.agent_type == 'greedy'
        assert config.greedy is True
        assert config.lookahead_depth == 1
        assert config.steps == 100
        assert config.ultra_fast is False
        assert config.memory_efficient is False
    
    def test_config_with_custom_values(self):
        """Should accept custom values."""
        config = WanderlineConfig(
            motif_path="test.png",
            ultra_fast=True,
            steps=500,
            opacity=0.3
        )
        
        assert config.motif_path == "test.png"
        assert config.ultra_fast is True
        assert config.steps == 500
        assert config.opacity == 0.3
    
    def test_auto_n_samples_selection(self):
        """Should auto-select n_samples based on lookahead_depth."""
        # Lookahead depth 1 -> 36 samples
        config1 = WanderlineConfig(lookahead_depth=1)
        assert config1.n_samples == 36
        
        # Lookahead depth 2 -> 16 samples  
        config2 = WanderlineConfig(lookahead_depth=2)
        assert config2.n_samples == 16
        
        # Lookahead depth 3+ -> 12 samples
        config3 = WanderlineConfig(lookahead_depth=3)
        assert config3.n_samples == 12
        
        # Manual n_samples should be preserved
        config4 = WanderlineConfig(lookahead_depth=1, n_samples=24)
        assert config4.n_samples == 24


class TestVisualizationLogic:
    """Test automatic visualization disabling logic."""
    
    def test_ultra_fast_disables_visualization(self):
        """Ultra-fast mode should disable visualization."""
        config = WanderlineConfig(ultra_fast=True)
        assert config.visualization['enabled'] is False
    
    def test_headless_disables_visualization(self):
        """Headless mode should disable visualization."""
        config = WanderlineConfig(headless=True)
        assert config.visualization['enabled'] is False
    
    def test_normal_mode_enables_visualization(self):
        """Normal mode should enable visualization by default."""
        config = WanderlineConfig()
        assert config.visualization['enabled'] is True
    
    def test_ultra_fast_prints_message(self, capsys):
        """Ultra-fast mode should print informational message."""
        WanderlineConfig(ultra_fast=True)
        captured = capsys.readouterr()
        assert "Ultra-fast mode: Real-time visualization automatically disabled" in captured.out


class TestConfigValidation:
    """Test configuration validation logic."""
    
    def test_valid_config_passes_validation(self):
        """Valid config should pass validation."""
        config = WanderlineConfig(
            ratio=0.5,
            steps=100,
            reward_type='l2',
            lookahead_depth=1
        )
        config.validate()  # Should not raise
    
    def test_invalid_ratio_fails_validation(self):
        """Invalid ratio should fail validation."""
        config = WanderlineConfig(ratio=0.0)
        with pytest.raises(ValueError, match="ratio must be > 0 and <= 1"):
            config.validate()
        
        config = WanderlineConfig(ratio=1.5)
        with pytest.raises(ValueError, match="ratio must be > 0 and <= 1"):
            config.validate()
    
    def test_negative_steps_fails_validation(self):
        """Negative steps should fail validation."""
        config = WanderlineConfig(steps=-1)
        with pytest.raises(ValueError, match="steps must be non-negative"):
            config.validate()
    
    def test_white_penalty_validation(self):
        """l2_white_penalty requires white_penalty value."""
        config = WanderlineConfig(reward_type='l2_white_penalty')
        with pytest.raises(ValueError, match="white_penalty required"):
            config.validate()
        
        # Should pass with white_penalty set
        config = WanderlineConfig(reward_type='l2_white_penalty', white_penalty=0.1)
        config.validate()  # Should not raise
    
    def test_fast_mode_requires_lookahead_1(self):
        """Fast modes should require lookahead_depth=1."""
        config = WanderlineConfig(ultra_fast=True, lookahead_depth=2)
        with pytest.raises(ValueError, match="Fast modes only work with lookahead_depth=1"):
            config.validate()
        
        config = WanderlineConfig(fast_agent=True, lookahead_depth=2)
        with pytest.raises(ValueError, match="Fast modes only work with lookahead_depth=1"):
            config.validate()


class TestIsFastModeProperty:
    """Test is_fast_mode property."""
    
    def test_ultra_fast_is_fast_mode(self):
        """Ultra-fast should be detected as fast mode."""
        config = WanderlineConfig(ultra_fast=True)
        assert config.is_fast_mode is True
    
    def test_fast_agent_is_fast_mode(self):
        """Fast agent should be detected as fast mode."""
        config = WanderlineConfig(fast_agent=True)
        assert config.is_fast_mode is True
    
    def test_normal_mode_is_not_fast_mode(self):
        """Normal mode should not be fast mode."""
        config = WanderlineConfig()
        assert config.is_fast_mode is False


class TestSafeResumeFunctionality:
    """Test safe resume without mutations."""
    
    def test_create_for_resume_creates_new_instance(self):
        """Should create new config instance, not mutate original."""
        original = WanderlineConfig(motif_path="original.png", steps=500)
        
        resume_data = {'motif_path': 'resumed.png', 'ratio': 0.3}
        resumed = original.create_for_resume(resume_data)
        
        # Original should be unchanged
        assert original.motif_path == "original.png"
        assert original.ratio == 0.2  # Default value
        
        # Resumed should have new values
        assert resumed.motif_path == "resumed.png"
        assert resumed.ratio == 0.3
        assert resumed.steps == 500  # Inherited from original
    
    def test_resume_handles_greedy_flag(self):
        """Should handle legacy greedy flag conversion."""
        original = WanderlineConfig(agent_type='greedy')
        
        resume_data = {'greedy': 'false'}
        resumed = original.create_for_resume(resume_data)
        
        assert resumed.greedy is False
        assert resumed.agent_type == 'transformer'
    
    def test_resume_handles_numeric_conversions(self):
        """Should handle string to numeric conversions."""
        original = WanderlineConfig()
        
        resume_data = {'ratio': '0.5', 'opacity': '0.8'}
        resumed = original.create_for_resume(resume_data)
        
        assert resumed.ratio == 0.5
        assert resumed.opacity == 0.8


class TestArgparseIntegration:
    """Test integration with argparse Namespace."""
    
    def test_from_args_basic_conversion(self):
        """Should convert argparse Namespace to config."""
        args = Namespace(
            motif_path="test.png",
            ultra_fast=True,
            steps=500,
            opacity=0.3,
            reward_type='l2_white_penalty',
            white_penalty=0.1
        )
        
        config = WanderlineConfig.from_args(args)
        
        assert config.motif_path == "test.png"
        assert config.ultra_fast is True
        assert config.steps == 500
        assert config.opacity == 0.3
        assert config.reward_type == 'l2_white_penalty'
        assert config.white_penalty == 0.1
    
    def test_from_args_handles_missing_fields(self):
        """Should handle args with missing fields gracefully."""
        args = Namespace(ultra_fast=True, steps=500)
        
        config = WanderlineConfig.from_args(args)
        
        assert config.ultra_fast is True
        assert config.steps == 500
        assert config.motif_path is None  # Default
        assert config.opacity == 1.0      # Default
    
    def test_to_args_namespace_conversion(self):
        """Should convert config back to argparse Namespace."""
        config = WanderlineConfig(
            motif_path="test.png",
            ultra_fast=True,
            steps=500
        )
        
        args = config.to_args_namespace()
        
        assert args.motif_path == "test.png"
        assert args.ultra_fast is True
        assert args.steps == 500


class TestConfigFactory:
    """Test config creation factory function."""
    
    def test_create_config_from_args_only(self):
        """Should create config from args only."""
        args = Namespace(ultra_fast=True, steps=500)
        config = create_config(args=args)
        
        assert config.ultra_fast is True
        assert config.steps == 500
    
    def test_create_config_from_json_only(self):
        """Should create config from JSON only."""
        json_data = {
            "motif_path": "test.png",
            "ultra_fast": True,
            "steps": 500
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(json_data, f)
            temp_path = f.name
        
        try:
            config = create_config(json_path=temp_path)
            
            assert config.motif_path == "test.png"
            assert config.ultra_fast is True
            assert config.steps == 500
        finally:
            os.unlink(temp_path)
    
    def test_create_config_args_override_json(self):
        """CLI args should override JSON values."""
        json_data = {"steps": 100, "ultra_fast": False}
        args = Namespace(steps=500, ultra_fast=True)
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(json_data, f)
            temp_path = f.name
        
        try:
            config = create_config(json_path=temp_path, args=args)
            
            assert config.steps == 500      # Args override
            assert config.ultra_fast is True # Args override
        finally:
            os.unlink(temp_path)
    
    def test_create_config_validates_result(self):
        """Factory should validate the created config."""
        args = Namespace(ultra_fast=True, lookahead_depth=2)  # Invalid combination
        
        with pytest.raises(ValueError, match="Fast modes only work with lookahead_depth=1"):
            create_config(args=args)


class TestLegacyCompatibility:
    """Test compatibility with existing codebase patterns."""
    
    def test_greedy_flag_sync_with_agent_type(self):
        """Greedy flag should sync with agent_type."""
        config1 = WanderlineConfig(agent_type='greedy')
        assert config1.greedy is True
        
        config2 = WanderlineConfig(agent_type='transformer')
        assert config2.greedy is False
    
    def test_visualization_dict_compatibility(self):
        """Should maintain visualization dict for backward compatibility."""
        config = WanderlineConfig()
        assert isinstance(config.visualization, dict)
        assert 'enabled' in config.visualization


if __name__ == "__main__":
    pytest.main([__file__, "-v"])