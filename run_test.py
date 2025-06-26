#!/usr/bin/env python3
"""
Wanderline main script: Generate single-stroke drawings that mimic a motif image.
"""

from wanderline.config_manager import load_config_and_parse_args, validate_args
from wanderline.run_manager import RunManager
from wanderline.drawing_engine import DrawingEngine


def main():
    """
    Main entry point for the Wanderline drawing system.
    """
    # Load configuration and parse arguments
    args = load_config_and_parse_args()
    validate_args(args)
    
    # Initialize managers
    run_manager = RunManager(args)
    drawing_engine = DrawingEngine(args)
    
    # Set up run (new or resumed)
    canvas, motif, current_start, initial_canvas = run_manager.setup_run()
    
    # Create output directory and save motif
    out_dir = run_manager.create_output_directory()
    run_manager.save_motif(motif)
    
    # Set up video recording
    recorder, frames_to_record, video_path = drawing_engine.setup_video_recording(canvas, out_dir)
    
    # Execute main drawing loop
    try:
        final_canvas, distances, total_reward, final_position, converged, executed_steps = drawing_engine.run_drawing_loop(
            canvas, motif, current_start, run_manager.initial_distances, recorder, frames_to_record, initial_canvas, out_dir, run_manager.previous_total_steps
        )
        
        # Finalize outputs (but skip video finalization if interrupted)
        if not drawing_engine.shutdown_requested:
            drawing_engine.finalize_outputs(distances, out_dir, video_path, recorder)
        else:
            # For interrupted runs, just plot distances
            if distances:
                from wanderline.plot_utils import plot_distances
                plot_distances(distances, out_dir)
        
        # Save all results
        run_manager.save_results(final_canvas, initial_canvas, distances, total_reward, final_position, converged, executed_steps)
        
        if drawing_engine.shutdown_requested:
            print("✅ Drawing interrupted but state saved successfully!")
            print(f"📁 Resume with: --resume_from {out_dir}")
        
    except KeyboardInterrupt:
        # Fallback in case signal handler doesn't work
        print("\n🛑 KeyboardInterrupt caught - saving current state...")
        if 'final_canvas' in locals():
            run_manager.save_results(final_canvas, initial_canvas, distances, total_reward, final_position, False, executed_steps)
            print("✅ Current state saved!")
            print(f"📁 Resume with: --resume_from {out_dir}")
        else:
            print("❌ Unable to save state - no progress to save")
        raise


if __name__ == "__main__":
    main()
