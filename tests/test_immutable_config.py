"""
Tests for immutable configuration system.
"""

import pytest
from wanderline.immutable_config import (
    ConfigBuilder, WanderlineConfig, AgentConfig, VisualizationConfig,
    RewardConfig, DrawingConfig, OptimizationConfig, ImmutableConfigError,
    create_config_from_sources, config_to_args_namespace
)
from argparse import Namespace
import tempfile
import json
import os


class TestAgentConfig:
    """Test agent configuration validation."""
    
    def test_valid_agent_config(self):
        config = AgentConfig(type='greedy', lookahead_depth=2, n_samples=16)
        assert config.type == 'greedy'
        assert config.lookahead_depth == 2
        assert config.n_samples == 16
    
    def test_invalid_lookahead_depth(self):
        with pytest.raises(ValueError, match="lookahead_depth must be >= 1"):
            AgentConfig(lookahead_depth=0)
    
    def test_invalid_n_samples(self):
        with pytest.raises(ValueError, match="n_samples must be > 0"):
            AgentConfig(n_samples=-1)
    
    def test_auto_samples_selection(self):
        config = AgentConfig(lookahead_depth=1)
        auto_config = config.with_auto_samples()
        assert auto_config.n_samples == 36
        
        config = AgentConfig(lookahead_depth=2)
        auto_config = config.with_auto_samples()
        assert auto_config.n_samples == 16
        
        config = AgentConfig(lookahead_depth=3)
        auto_config = config.with_auto_samples()
        assert auto_config.n_samples == 12
    
    def test_is_fast_mode(self):
        config = AgentConfig(fast_mode=True)
        assert config.is_fast_mode
        
        config = AgentConfig(ultra_fast=True)
        assert config.is_fast_mode
        
        config = AgentConfig(fast_mode=False, ultra_fast=False)
        assert not config.is_fast_mode


class TestRewardConfig:
    """Test reward configuration validation."""
    
    def test_l2_reward_valid(self):
        config = RewardConfig(type='l2')
        assert config.type == 'l2'
        assert config.white_penalty is None
    
    def test_white_penalty_required(self):
        with pytest.raises(ValueError, match="white_penalty required"):
            RewardConfig(type='l2_white_penalty', white_penalty=None)
    
    def test_white_penalty_valid_range(self):
        config = RewardConfig(type='l2_white_penalty', white_penalty=0.1)
        assert config.white_penalty == 0.1
        
        with pytest.raises(ValueError, match="white_penalty must be 0-1"):
            RewardConfig(type='l2_white_penalty', white_penalty=1.5)


class TestDrawingConfig:
    """Test drawing configuration validation."""
    
    def test_valid_drawing_config(self):
        config = DrawingConfig(steps=500, opacity=0.5, ratio=0.2)
        assert config.steps == 500
        assert config.opacity == 0.5
        assert config.ratio == 0.2
    
    def test_invalid_steps(self):
        with pytest.raises(ValueError, match="steps must be non-negative"):
            DrawingConfig(steps=-1)
    
    def test_invalid_ratio(self):
        with pytest.raises(ValueError, match="ratio must be > 0 and <= 1"):
            DrawingConfig(ratio=0.0)
        
        with pytest.raises(ValueError, match="ratio must be > 0 and <= 1"):
            DrawingConfig(ratio=1.5)
    
    def test_invalid_opacity(self):
        with pytest.raises(ValueError, match="opacity must be 0-1"):
            DrawingConfig(opacity=1.5)


class TestConfigBuilder:
    """Test configuration builder functionality."""
    
    def test_basic_builder_usage(self):
        config = (ConfigBuilder()
                  .motif_path("test.png")
                  .agent(ultra_fast=True)
                  .drawing(steps=500)
                  .freeze())
        
        assert config.motif_path == "test.png"
        assert config.agent.ultra_fast is True
        assert config.drawing.steps == 500
    
    def test_freeze_makes_immutable(self):
        builder = ConfigBuilder()
        config = builder.freeze()
        
        # Builder should not be usable after freeze
        with pytest.raises(ImmutableConfigError):
            builder.agent(type='transformer')
    
    def test_from_json(self):
        json_data = {
            "motif_path": "assets/test.png",
            "agent": {"type": "greedy", "ultra_fast": True},
            "drawing": {"steps": 1000, "opacity": 0.3}
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(json_data, f)
            temp_path = f.name
        
        try:
            config = (ConfigBuilder()
                      .from_json(temp_path)
                      .freeze())
            
            assert config.motif_path == "assets/test.png"
            assert config.agent.type == "greedy"
            assert config.agent.ultra_fast is True
            assert config.drawing.steps == 1000
            assert config.drawing.opacity == 0.3
        finally:
            os.unlink(temp_path)
    
    def test_from_args(self):
        args = Namespace(
            motif_path="test.png",
            ultra_fast=True,
            steps=500,
            opacity=0.3,
            reward_type='l2_white_penalty',
            white_penalty=0.1,
            memory_efficient=True
        )
        
        config = (ConfigBuilder()
                  .from_args(args)
                  .freeze())
        
        assert config.motif_path == "test.png"
        assert config.agent.ultra_fast is True
        assert config.drawing.steps == 500
        assert config.drawing.opacity == 0.3
        assert config.reward.type == 'l2_white_penalty'
        assert config.reward.white_penalty == 0.1
        assert config.optimization.memory_efficient is True


class TestWanderlineConfig:
    """Test main configuration class."""
    
    def test_ultra_fast_disables_visualization(self):
        config = (ConfigBuilder()
                  .agent(ultra_fast=True)
                  .visualization(enabled=True)
                  .freeze())
        
        # Ultra-fast should override visualization.enabled
        assert config.visualization.enabled is False
    
    def test_headless_disables_visualization(self):
        config = (ConfigBuilder()
                  .optimization(headless=True)
                  .visualization(enabled=True)
                  .freeze())
        
        assert config.visualization.enabled is False
    
    def test_fast_mode_requires_lookahead_1(self):
        with pytest.raises(ValueError, match="Fast modes only work with lookahead_depth=1"):
            (ConfigBuilder()
             .agent(ultra_fast=True, lookahead_depth=2)
             .freeze())
    
    def test_config_is_immutable(self):
        config = (ConfigBuilder()
                  .drawing(steps=500)
                  .freeze())
        
        # Should not be able to modify frozen config
        with pytest.raises(AttributeError):
            config.drawing.steps = 1000
    
    def test_for_resume(self):
        config = (ConfigBuilder()
                  .motif_path("original.png")
                  .drawing(steps=500, opacity=0.5)
                  .freeze())
        
        resume_data = {
            'motif_path': 'resumed.png',
            'steps': 750,
            'opacity': 0.3
        }
        
        resumed_config = config.for_resume(resume_data)
        
        # Original config unchanged
        assert config.motif_path == "original.png"
        assert config.drawing.steps == 500
        assert config.drawing.opacity == 0.5
        
        # New config has overrides
        assert resumed_config.motif_path == "resumed.png"
        assert resumed_config.drawing.steps == 750
        assert resumed_config.drawing.opacity == 0.3
    
    def test_to_dict(self):
        config = (ConfigBuilder()
                  .motif_path("test.png")
                  .agent(ultra_fast=True)
                  .drawing(steps=500)
                  .freeze())
        
        config_dict = config.to_dict()
        
        assert config_dict['motif_path'] == "test.png"
        assert config_dict['agent']['ultra_fast'] is True
        assert config_dict['drawing']['steps'] == 500


class TestConfigConversion:
    """Test conversion functions."""
    
    def test_config_to_args_namespace(self):
        config = (ConfigBuilder()
                  .motif_path("test.png")
                  .agent(type='greedy', ultra_fast=True, lookahead_depth=1)
                  .drawing(steps=500, opacity=0.3)
                  .reward(type='l2_white_penalty', white_penalty=0.1)
                  .freeze())
        
        args = config_to_args_namespace(config)
        
        assert args.motif_path == "test.png"
        assert args.agent_type == 'greedy'
        assert args.ultra_fast is True
        assert args.greedy is True  # Legacy compatibility
        assert args.steps == 500
        assert args.opacity == 0.3
        assert args.reward_type == 'l2_white_penalty'
        assert args.white_penalty == 0.1
    
    def test_create_config_from_sources(self):
        args = Namespace(ultra_fast=True, steps=500)
        config = create_config_from_sources(args=args)
        
        assert config.agent.ultra_fast is True
        assert config.drawing.steps == 500


class TestConfigLifecycle:
    """Test configuration lifecycle management."""
    
    def test_construction_phase_allows_mutations(self):
        builder = ConfigBuilder()
        builder.agent(type='greedy')
        builder.agent(ultra_fast=True)  # Should work
        
        config = builder.freeze()
        assert config.agent.type == 'greedy'
        assert config.agent.ultra_fast is True
    
    def test_frozen_phase_prevents_mutations(self):
        builder = ConfigBuilder()
        config = builder.freeze()
        
        # Should not be able to use builder after freeze
        with pytest.raises(ImmutableConfigError):
            builder.agent(type='transformer')


if __name__ == "__main__":
    pytest.main([__file__])