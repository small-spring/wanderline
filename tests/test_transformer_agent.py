"""
Tests for transformer agent implementation.
"""

import numpy as np
import torch
from wanderline.transformer_agent import TransformerAgent, GreedyAgent, StateEncoder


class TestStateEncoder:
    """Test the state encoder component."""
    
    def test_state_encoder_initialization(self):
        """Test that StateEncoder initializes correctly."""
        encoder = StateEncoder((100, 100), feature_dim=128)
        assert encoder.feature_dim == 128
        assert encoder.canvas_shape == (100, 100)
    
    def test_state_encoder_forward(self):
        """Test forward pass of StateEncoder."""
        encoder = StateEncoder((64, 64), feature_dim=64)
        
        # Create dummy inputs
        batch_size = 2
        canvas = torch.randn(batch_size, 3, 64, 64)
        motif = torch.randn(batch_size, 3, 64, 64)
        position = torch.randn(batch_size, 2)
        history = torch.randn(batch_size, 10)
        
        # Forward pass
        features = encoder(canvas, motif, position, history)
        
        assert features.shape == (batch_size, 64)
        assert not torch.isnan(features).any()


class TestTransformerAgent:
    """Test the TransformerAgent implementation."""
    
    def test_transformer_agent_initialization(self):
        """Test that TransformerAgent initializes correctly."""
        agent = TransformerAgent((100, 100), sequence_length=5, feature_dim=64)
        assert agent.canvas_shape == (100, 100)
        assert agent.sequence_length == 5
        assert agent.feature_dim == 64
        assert len(agent.action_history) == 0
    
    def test_choose_next_angle(self):
        """Test angle selection."""
        agent = TransformerAgent((64, 64), feature_dim=32)
        
        # Create dummy canvas and motif
        canvas = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        motif = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        start_pt = (32, 32)
        length = 10
        
        angle = agent.choose_next_angle(canvas, motif, start_pt, length)
        
        assert isinstance(angle, float)
        assert 0 <= angle < 2 * np.pi
        assert len(agent.action_history) == 1
    
    def test_action_history_management(self):
        """Test that action history is managed correctly."""
        agent = TransformerAgent((64, 64), feature_dim=32)
        
        canvas = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        motif = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        start_pt = (32, 32)
        length = 10
        
        # Generate more than 10 actions
        for i in range(15):
            agent.choose_next_angle(canvas, motif, start_pt, length)
        
        # History should be limited to 10
        assert len(agent.action_history) == 10
    
    def test_pad_history(self):
        """Test history padding functionality."""
        agent = TransformerAgent((64, 64))
        
        # Empty history
        padded = agent._pad_history()
        assert len(padded) == 10
        assert all(x == 0.0 for x in padded)
        
        # Partial history
        agent.action_history = [1.0, 2.0, 3.0]
        padded = agent._pad_history()
        assert len(padded) == 10
        assert padded[-3:] == [1.0, 2.0, 3.0]
        assert padded[:7] == [0.0] * 7


class TestGreedyAgent:
    """Test the refactored GreedyAgent for baseline comparison."""
    
    def test_greedy_agent_initialization(self):
        """Test that GreedyAgent initializes correctly."""
        agent = GreedyAgent()
        assert agent is not None
    
    def test_choose_next_angle_deterministic(self):
        """Test that greedy agent is deterministic."""
        agent = GreedyAgent()
        
        # Create simple test case
        canvas = np.ones((64, 64, 3), dtype=np.uint8) * 255  # White canvas
        motif = np.zeros((64, 64, 3), dtype=np.uint8)  # Black motif
        start_pt = (32, 32)
        length = 10
        
        # Should get same angle for same input
        angle1 = agent.choose_next_angle(canvas, motif, start_pt, length, n_samples=8)
        angle2 = agent.choose_next_angle(canvas, motif, start_pt, length, n_samples=8)
        
        assert angle1 == angle2
        assert isinstance(angle1, float)
        assert 0 <= angle1 < 2 * np.pi


class TestIntegration:
    """Integration tests comparing agents."""
    
    def test_agents_produce_valid_angles(self):
        """Test that both agents produce valid angles."""
        transformer_agent = TransformerAgent((64, 64), feature_dim=32)
        greedy_agent = GreedyAgent()
        
        canvas = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        motif = np.random.randint(0, 256, (64, 64, 3), dtype=np.uint8)
        start_pt = (32, 32)
        length = 10
        
        transformer_angle = transformer_agent.choose_next_angle(canvas, motif, start_pt, length)
        greedy_angle = greedy_agent.choose_next_angle(canvas, motif, start_pt, length, n_samples=8)
        
        # Both should produce valid angles
        assert 0 <= transformer_angle < 2 * np.pi
        assert 0 <= greedy_angle < 2 * np.pi
        
        # They should be different (transformer uses neural net, greedy uses search)
        assert transformer_angle != greedy_angle
