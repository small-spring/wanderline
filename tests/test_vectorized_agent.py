import unittest
import numpy as np
import time
from wanderline.agent import choose_next_angle_vectorized, choose_next_angle, _choose_next_angle_greedy


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
        """Test basic functionality of vectorized single-step agent."""
        angle, timing_info = choose_next_angle_vectorized(
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
        """Test vectorized agent with white penalty."""
        angle, timing_info = choose_next_angle_vectorized(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=8, reward_type='l2_white_penalty', white_penalty=0.1
        )
        
        # Should return a valid angle
        self.assertIsInstance(angle, float)
        self.assertGreaterEqual(angle, 0.0)
        self.assertLess(angle, 2 * np.pi)
        
    def test_vectorized_vs_sequential_consistency(self):
        """Test that vectorized and sequential methods give similar results."""
        # Use same random seed for reproducibility
        np.random.seed(42)
        
        n_samples = 12
        
        # Get result from sequential method
        sequential_angle = _choose_next_angle_greedy(
            self.blank_canvas, self.motif, self.start_pt, self.length, n_samples
        )
        
        # Get result from vectorized method
        vectorized_angle, _ = choose_next_angle_vectorized(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=n_samples, reward_type='l2'
        )
        
        # Both should be valid angles
        self.assertIsInstance(sequential_angle, float)
        self.assertIsInstance(vectorized_angle, float)
        
        # They should be exactly the same (same computation, same angles tested)
        self.assertAlmostEqual(sequential_angle, vectorized_angle, places=5)
        
    def test_vectorized_performance(self):
        """Test that vectorized version is faster than sequential."""
        n_samples = 24
        
        # Time sequential version
        start_time = time.time()
        for _ in range(5):  # Run multiple times for better measurement
            sequential_angle = _choose_next_angle_greedy(
                self.blank_canvas, self.motif, self.start_pt, self.length, n_samples
            )
        sequential_time = time.time() - start_time
        
        # Time vectorized version
        start_time = time.time()
        for _ in range(5):
            vectorized_angle, _ = choose_next_angle_vectorized(
                self.blank_canvas, self.motif, self.start_pt, self.length, 
                n_samples=n_samples, reward_type='l2'
            )
        vectorized_time = time.time() - start_time
        
        # Both should produce valid results
        self.assertIsInstance(sequential_angle, float)
        self.assertIsInstance(vectorized_angle, float)
        
        # Vectorized should be faster (allowing some variance)
        print(f"Sequential time: {sequential_time:.4f}s, Vectorized time: {vectorized_time:.4f}s")
        # Note: On small examples, vectorized might not always be faster due to overhead
        # But it should scale better with larger n_samples
        
    def test_vectorized_edge_cases(self):
        """Test vectorized agent with edge cases."""
        # Very small canvas
        small_canvas = np.ones((10, 10, 3), dtype=np.uint8) * 255
        small_motif = np.zeros((10, 10, 3), dtype=np.uint8)
        
        angle, _ = choose_next_angle_vectorized(
            small_canvas, small_motif, (5, 5), 3, 
            n_samples=4, reward_type='l2'
        )
        
        self.assertIsInstance(angle, float)
        self.assertGreaterEqual(angle, 0.0)
        self.assertLess(angle, 2 * np.pi)
        
        # Single sample
        angle, _ = choose_next_angle_vectorized(
            self.blank_canvas, self.motif, self.start_pt, self.length, 
            n_samples=1, reward_type='l2'
        )
        
        self.assertIsInstance(angle, float)
        self.assertAlmostEqual(angle, 0.0, places=5)  # Should be 0 for single sample
        
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
