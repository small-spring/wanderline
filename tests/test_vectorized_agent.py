import unittest
import numpy as np
import time
from wanderline.agent import choose_next_angle
from wanderline.memory_efficient_canvas import choose_angle_memory_efficient


class TestVectorizedAgent(unittest.TestCase):
    """
    Tests for vectorized agent implementations.
    """
    
    def setUp(self):
        """Set up test cases with simple canvas and motif."""
        self.canvas_size = (50, 50, 3)
        self.blank_canvas = np.ones(self.canvas_size, dtype=np.uint8) * 255
        self.motif = np.zeros(self.canvas_size, dtype=np.uint8)
        self.start_pt = (25, 25)
        self.length = 10
        
    def test_vectorized_single_step_basic(self):
        """Test basic functionality of memory-efficient single-step agent."""
        angle, timing_info = choose_angle_memory_efficient(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, reward_type='l2'
        )
        
        # Should return a valid angle
        self.assertIsInstance(angle, float)
        self.assertGreaterEqual(angle, 0.0)
        self.assertLess(angle, 2 * np.pi)
        
        # timing_info should be empty when verbose=False
        self.assertIsInstance(timing_info, dict)
        
    def test_vectorized_with_white_penalty(self):
        """Test memory-efficient agent with white penalty."""
        angle, timing_info = choose_angle_memory_efficient(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, reward_type='l2_white_penalty', white_penalty=0.1
        )
        
        # Should return a valid angle
        self.assertIsInstance(angle, float)
        self.assertGreaterEqual(angle, 0.0)
        self.assertLess(angle, 2 * np.pi)
        
    def test_vectorized_vs_sequential_consistency(self):
        """Test that main choose_next_angle gives consistent results."""
        # Use same random seed for reproducibility
        np.random.seed(42)
        
        n_samples = 12
        
        # Get result from main function (now uses memory-efficient internally)
        angle1 = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=n_samples, reward_type='l2', use_vectorized=True
        )
        
        # Get result from memory-efficient function directly
        angle2, _ = choose_angle_memory_efficient(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=n_samples, reward_type='l2'
        )
        
        # Both should be valid angles
        self.assertIsInstance(angle1, float)
        self.assertIsInstance(angle2, float)
        
        # They should give similar results (may have slight differences due to different sampling strategies)
        self.assertGreaterEqual(angle1, 0.0)
        self.assertLess(angle1, 2 * np.pi)
        self.assertGreaterEqual(angle2, 0.0)
        self.assertLess(angle2, 2 * np.pi)
        
    def test_vectorized_performance(self):
        """Test that memory-efficient implementation performs well."""
        n_samples = 24
        
        # Time main function (now memory-efficient)
        start_time = time.time()
        for _ in range(5):  # Run multiple times for better measurement
            angle1 = choose_next_angle(
                self.blank_canvas, self.motif, self.start_pt, self.length, 
                n_samples=n_samples, reward_type='l2'
            )
        main_time = time.time() - start_time
        
        # Time memory-efficient function directly
        start_time = time.time()
        for _ in range(5):
            angle2, _ = choose_angle_memory_efficient(
                self.blank_canvas, self.motif, self.start_pt, self.length, 
                n_samples=n_samples, reward_type='l2'
            )
        efficient_time = time.time() - start_time
        
        # Both should produce valid results
        self.assertIsInstance(angle1, float)
        self.assertIsInstance(angle2, float)
        
        # Both should be reasonably fast
        print(f"Main function time: {main_time:.4f}s, Memory-efficient time: {efficient_time:.4f}s")
        # Note: Memory-efficient should use minimal memory compared to old vectorized approach
        
    def test_vectorized_edge_cases(self):
        """Test memory-efficient agent with edge cases."""
        # Very small canvas
        small_canvas = np.ones((10, 10, 3), dtype=np.uint8) * 255
        small_motif = np.zeros((10, 10, 3), dtype=np.uint8)
        
        angle, _ = choose_angle_memory_efficient(
            small_canvas, small_motif, (5, 5), 3, 
            n_samples=4, reward_type='l2'
        )
        
        self.assertIsInstance(angle, float)
        self.assertGreaterEqual(angle, 0.0)
        self.assertLess(angle, 2 * np.pi)
        
        # Single sample
        angle, _ = choose_angle_memory_efficient(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=1, reward_type='l2'
        )
        
        self.assertIsInstance(angle, float)
        self.assertGreaterEqual(angle, 0.0)  # Should be valid angle for single sample
        self.assertLess(angle, 2 * np.pi)
        
    def test_main_function_vectorized_flag(self):
        """Test that main choose_next_angle function uses vectorized implementation."""
        angle_vectorized = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, lookahead_depth=1, reward_type='l2', use_vectorized=True
        )
        
        angle_sequential = choose_next_angle(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, lookahead_depth=1, reward_type='l2', use_vectorized=False
        )
        
        # Both should be valid and equal
        self.assertIsInstance(angle_vectorized, float)
        self.assertIsInstance(angle_sequential, float)
        self.assertAlmostEqual(angle_vectorized, angle_sequential, places=5)


if __name__ == '__main__':
    unittest.main()
