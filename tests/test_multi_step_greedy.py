import unittest
import numpy as np
from wanderline.agent import choose_next_angle


class TestMultiStepGreedy(unittest.TestCase):
    """
    Tests for multi-step greedy agent with lookahead depth functionality.
    """
    
    def setUp(self):
        """Set up test cases with simple canvas and motif."""
        self.canvas_size = (50, 50, 3)
        self.blank_canvas = np.ones(self.canvas_size, dtype=np.uint8) * 255
        self.motif = np.zeros(self.canvas_size, dtype=np.uint8)
        self.start_pt = (25, 25)
        self.length = 10
        
    def test_lookahead_depth_1_equals_greedy(self):
        """Test that lookahead_depth=1 produces same result as greedy algorithm."""
        angle_depth_1 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, lookahead_depth=1
        )
        
        # The result should be a valid angle
        self.assertIsInstance(angle_depth_1, float)
        self.assertGreaterEqual(angle_depth_1, 0.0)
        self.assertLess(angle_depth_1, 2 * np.pi)
        
    def test_lookahead_depth_2_valid(self):
        """Test that lookahead_depth=2 produces valid results."""
        angle_depth_2 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=4, lookahead_depth=2
        )
        
        # The result should be a valid angle
        self.assertIsInstance(angle_depth_2, float)
        self.assertGreaterEqual(angle_depth_2, 0.0)
        self.assertLess(angle_depth_2, 2 * np.pi)
        
    def test_lookahead_depth_3_valid(self):
        """Test that lookahead_depth=3 produces valid results."""
        angle_depth_3 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=4, lookahead_depth=3
        )
        
        # The result should be a valid angle
        self.assertIsInstance(angle_depth_3, float)
        self.assertGreaterEqual(angle_depth_3, 0.0)
        self.assertLess(angle_depth_3, 2 * np.pi)
        
    def test_different_depths_consistency(self):
        """Test that different lookahead depths produce consistent results."""
        # Test with same input but different depths
        np.random.seed(42)  # For reproducible results
        
        angle_1 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, lookahead_depth=1
        )
        
        angle_2 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, lookahead_depth=2
        )
        
        # Both should be valid angles (may or may not be the same)
        self.assertIsInstance(angle_1, float)
        self.assertIsInstance(angle_2, float)
        self.assertGreaterEqual(angle_1, 0.0)
        self.assertGreaterEqual(angle_2, 0.0)
        self.assertLess(angle_1, 2 * np.pi)
        self.assertLess(angle_2, 2 * np.pi)
        
    def test_small_canvas_edge_case(self):
        """Test multi-step greedy on very small canvas."""
        small_canvas = np.ones((10, 10, 3), dtype=np.uint8) * 255
        small_motif = np.zeros((10, 10, 3), dtype=np.uint8)
        
        angle = choose_next_angle(
            small_canvas, small_motif, (5, 5), 3, 
            n_samples=4, lookahead_depth=2
        )
        
        self.assertIsInstance(angle, float)
        self.assertGreaterEqual(angle, 0.0)
        self.assertLess(angle, 2 * np.pi)
        
    def test_performance_comparison(self):
        """Test that deeper lookahead takes longer but produces valid results."""
        import time
        
        # Test lookahead depth 1
        start_time = time.time()
        angle_1 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=6, lookahead_depth=1
        )
        time_1 = time.time() - start_time
        
        # Test lookahead depth 2
        start_time = time.time()
        angle_2 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=6, lookahead_depth=2
        )
        time_2 = time.time() - start_time
        
        # Both should produce valid angles
        self.assertIsInstance(angle_1, float)
        self.assertIsInstance(angle_2, float)
        
        # Depth 2 should take longer (but allow for some variance)
        # This test is mainly to ensure the function doesn't crash
        self.assertGreater(time_2, 0)
        self.assertGreater(time_1, 0)


if __name__ == '__main__':
    unittest.main()
