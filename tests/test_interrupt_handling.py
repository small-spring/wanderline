#!/usr/bin/env python3
"""
Tests for interrupt handling functionality in DrawingEngine.
"""

import os
import signal
import time
import threading
import tempfile
import shutil
import pytest
import sys
from unittest.mock import patch

from wanderline.config_manager import load_config_and_parse_args, validate_args
from wanderline.run_manager import RunManager
from wanderline.drawing_engine import DrawingEngine


class TestInterruptHandling:
    """Test interrupt handling in DrawingEngine."""
    
    def setup_method(self):
        """Set up test environment."""
        self.test_output_dir = tempfile.mkdtemp(prefix="wanderline_test_")
        
    def teardown_method(self):
        """Clean up test environment."""
        if os.path.exists(self.test_output_dir):
            shutil.rmtree(self.test_output_dir)
    
    def create_test_args(self, steps=100):
        """Create test arguments for DrawingEngine."""
        # Mock sys.argv to avoid parsing actual command line args
        with patch.object(sys, 'argv', ['test', '--steps', str(steps), '--greedy', '--opacity', '0.5', '--headless']):
            args = load_config_and_parse_args()
            validate_args(args)
            return args
    
    def test_signal_handler_setup(self):
        """Test that signal handlers are properly set up."""
        args = self.create_test_args()
        drawing_engine = DrawingEngine(args)
        
        # Check that shutdown_requested is initialized
        assert hasattr(drawing_engine, 'shutdown_requested')
        assert drawing_engine.shutdown_requested is False
        
        # Check that signal handlers are set up (by verifying they're not default)
        current_sigint_handler = signal.signal(signal.SIGINT, signal.SIG_DFL)
        current_sigterm_handler = signal.signal(signal.SIGTERM, signal.SIG_DFL)
        
        # Restore original handlers
        signal.signal(signal.SIGINT, current_sigint_handler)
        signal.signal(signal.SIGTERM, current_sigterm_handler)
        
        # The handlers should not be the default
        assert current_sigint_handler != signal.SIG_DFL
        assert current_sigterm_handler != signal.SIG_DFL
    
    def test_interrupt_signal_handler(self):
        """Test that interrupt signal sets shutdown flag."""
        args = self.create_test_args()
        drawing_engine = DrawingEngine(args)
        
        # Initially, shutdown should not be requested
        assert drawing_engine.shutdown_requested is False
        
        # Simulate SIGINT signal
        drawing_engine._handle_interrupt(signal.SIGINT, None)
        
        # Shutdown should now be requested
        assert drawing_engine.shutdown_requested is True
    
    def test_graceful_shutdown_in_drawing_loop(self):
        """Test that drawing loop respects shutdown request."""
        args = self.create_test_args(steps=1000)  # Long run
        
        # Initialize managers
        run_manager = RunManager(args)
        drawing_engine = DrawingEngine(args)
        
        # Set up run
        canvas, motif, current_start, initial_canvas = run_manager.setup_run()
        
        # Create temporary output directory for this test
        out_dir = os.path.join(self.test_output_dir, "test_run")
        os.makedirs(out_dir, exist_ok=True)
        
        # Set up video recording
        recorder, frames_to_record, video_path = drawing_engine.setup_video_recording(canvas, out_dir)
        
        # Function to request shutdown after a short delay
        def request_shutdown():
            time.sleep(0.1)  # Very short delay
            drawing_engine.shutdown_requested = True
        
        # Start shutdown request in background
        shutdown_thread = threading.Thread(target=request_shutdown)
        shutdown_thread.daemon = True
        shutdown_thread.start()
        
        # Execute drawing loop
        final_canvas, distances, total_reward, final_position, converged, executed_steps = drawing_engine.run_drawing_loop(
            canvas, motif, current_start, run_manager.initial_distances, recorder, frames_to_record, initial_canvas, out_dir, run_manager.previous_total_steps
        )
        
        # Verify that shutdown was requested and loop exited early
        assert drawing_engine.shutdown_requested is True
        assert executed_steps < args.steps  # Should have stopped early
        assert executed_steps > 0  # But should have made some progress
        
        # Verify that state can be saved
        assert final_canvas is not None
        assert len(distances) > 0
        assert final_position is not None
    
    @pytest.mark.integration
    def test_full_interrupt_workflow(self):
        """Integration test for full interrupt and resume workflow."""
        args = self.create_test_args(steps=50)
        
        # Initialize managers
        run_manager = RunManager(args)
        drawing_engine = DrawingEngine(args)
        
        # Set up run
        canvas, motif, current_start, initial_canvas = run_manager.setup_run()
        
        # Create output directory
        out_dir = os.path.join(self.test_output_dir, "integration_test")
        os.makedirs(out_dir, exist_ok=True)
        
        # Set run_manager's output directory to use our test directory
        run_manager.out_dir = out_dir
        
        # Save motif for later resume
        run_manager.save_motif(motif)
        
        # Set up video recording
        recorder, frames_to_record, video_path = drawing_engine.setup_video_recording(canvas, out_dir)
        
        # Function to interrupt after some steps
        def interrupt_after_steps():
            time.sleep(0.2)  # Allow some steps to execute
            drawing_engine.shutdown_requested = True
        
        interrupt_thread = threading.Thread(target=interrupt_after_steps)
        interrupt_thread.daemon = True
        interrupt_thread.start()
        
        # Execute drawing loop
        final_canvas, distances, total_reward, final_position, converged, executed_steps = drawing_engine.run_drawing_loop(
            canvas, motif, current_start, run_manager.initial_distances, recorder, frames_to_record, initial_canvas, out_dir, run_manager.previous_total_steps
        )
        
        # Save results
        run_manager.save_results(final_canvas, initial_canvas, distances, total_reward, final_position, converged, executed_steps)
        
        # Verify state was saved
        run_summary_path = os.path.join(out_dir, "run_summary.json")
        assert os.path.exists(run_summary_path)
        
        final_canvas_path = os.path.join(out_dir, "final_canvas.png")
        assert os.path.exists(final_canvas_path)
        
        # Verify that the run was interrupted (not completed)
        assert executed_steps < args.steps
        assert drawing_engine.shutdown_requested is True
    
    def test_no_interrupt_completes_normally(self):
        """Test that normal runs without interrupts complete successfully."""
        args = self.create_test_args(steps=5)  # Very short run
        
        # Initialize managers
        run_manager = RunManager(args)
        drawing_engine = DrawingEngine(args)
        
        # Set up run
        canvas, motif, current_start, initial_canvas = run_manager.setup_run()
        
        # Create output directory
        out_dir = os.path.join(self.test_output_dir, "normal_test")
        os.makedirs(out_dir, exist_ok=True)
        
        # Set up video recording
        recorder, frames_to_record, video_path = drawing_engine.setup_video_recording(canvas, out_dir)
        
        # Execute drawing loop without interruption
        final_canvas, distances, total_reward, final_position, converged, executed_steps = drawing_engine.run_drawing_loop(
            canvas, motif, current_start, run_manager.initial_distances, recorder, frames_to_record, initial_canvas, out_dir, run_manager.previous_total_steps
        )
        
        # Verify normal completion
        assert drawing_engine.shutdown_requested is False  # No interrupt requested
        assert executed_steps == args.steps  # All steps completed
        assert final_canvas is not None
        assert len(distances) == args.steps + 1  # Initial + all steps