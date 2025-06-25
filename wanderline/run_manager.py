import os
import sys
import json
import cv2
import numpy as np
import datetime
from wanderline.image_utils import load_image


class RunManager:
    """
    Manages run setup, resumption, and output handling.
    """
    
    def __init__(self, args):
        self.args = args
        self.resumed_from_path = None
        self.previous_total_steps = 0
        self.initial_distances = []
        self.out_dir = None
        
    def setup_run(self):
        """
        Set up a new run or resume from previous state.
        Returns: (canvas, motif, current_start, initial_canvas)
        """
        if self.args.resume_from:
            return self._resume_run()
        else:
            return self._setup_new_run()
    
    def _resume_run(self):
        """
        Resume from a previous run.
        Returns: (canvas, motif, current_start, initial_canvas)
        """
        self.resumed_from_path = self.args.resume_from
        print(f"Resuming from: {self.resumed_from_path}")
        
        # Load previous summary
        resume_summary_path = os.path.join(self.resumed_from_path, 'run_summary.json')
        if not os.path.isfile(resume_summary_path):
            print(f"Error: Cannot find 'run_summary.json' in {self.resumed_from_path}", file=sys.stderr)
            sys.exit(1)
        with open(resume_summary_path, 'r') as f:
            resume_summary = json.load(f)
        
        # Override args with previous config for consistency
        original_config = resume_summary['config']
        self.args.motif_path = original_config.get('motif_path')
        self.args.ratio = float(original_config.get('ratio'))
        self.args.opacity = float(original_config.get('opacity'))
        self.args.greedy = str(original_config.get('greedy')).lower() == 'true'
        
        # Load previous canvas
        resume_canvas_path = os.path.join(self.resumed_from_path, 'final_canvas.png')
        if not os.path.isfile(resume_canvas_path):
            print(f"Error: Cannot find 'final_canvas.png' in {self.resumed_from_path}", file=sys.stderr)
            sys.exit(1)
        canvas = cv2.imread(resume_canvas_path)
        if canvas is None:
            print(f"Error: Failed to load canvas from {resume_canvas_path}", file=sys.stderr)
            sys.exit(1)
        
        # Load last position
        if 'results' in resume_summary and 'last_position' in resume_summary['results']:
            current_start = tuple(resume_summary['results']['last_position'])
        else:
            print("Error: 'last_position' not found in summary. Cannot resume.", file=sys.stderr)
            sys.exit(1)
        
        # Load previous distances and step counts
        resume_distances_path = os.path.join(self.resumed_from_path, 'distances.json')
        if os.path.isfile(resume_distances_path):
            with open(resume_distances_path, 'r') as f:
                self.initial_distances = json.load(f)
            self.previous_total_steps = len(self.initial_distances) - 1 if self.initial_distances else 0
        else:
            print("Warning: 'distances.json' not found. Loss curve will be based on this run only.")
            if 'results' in resume_summary:
                if 'final_distance' in resume_summary['results']:
                    self.initial_distances = [resume_summary['results']['final_distance']]
                if 'total_steps_from_start' in resume_summary['results']:
                    self.previous_total_steps = resume_summary['results']['total_steps_from_start']
                elif 'total_steps_executed' in resume_summary['results']:
                    self.previous_total_steps = resume_summary['results']['total_steps_executed']
        
        # Load the motif image for the resumed run
        motif = None
        if self.args.motif_path:
            try:
                motif_rgb = load_image(self.args.motif_path)  # Returns RGB
                motif = cv2.cvtColor(motif_rgb, cv2.COLOR_RGB2BGR)  # Convert to BGR for CV2 ops
            except Exception as e:
                print(f"Error loading motif image for resumed run: {e}", file=sys.stderr)
                sys.exit(1)
        
        # Create initial canvas for output
        initial_canvas = np.ones_like(canvas, dtype=np.uint8) * 255
        
        return canvas, motif, current_start, initial_canvas
    
    def _setup_new_run(self):
        """
        Set up a new run from scratch.
        Returns: (canvas, motif, current_start, initial_canvas)
        """
        # Load motif (as RGB first)
        motif_rgb = None
        if self.args.motif_path:
            if not os.path.isfile(self.args.motif_path):
                print(f"Error: motif file '{self.args.motif_path}' does not exist", file=sys.stderr)
                sys.exit(1)
            try:
                # load_image returns RGB, used for initial analysis (e.g., finding darkest point)
                motif_rgb = load_image(self.args.motif_path)
            except Exception as e:
                print(f"Error loading motif image: {e}", file=sys.stderr)
                sys.exit(1)
        
        # Initialize canvas (as RGB) and determine start point
        if motif_rgb is not None:
            # Canvas is white, same size as motif. Stays in RGB for now.
            canvas_rgb = np.ones_like(motif_rgb, dtype=np.uint8) * 255
            # Find darkest point from a grayscale version of the RGB motif
            gray_motif = cv2.cvtColor(motif_rgb, cv2.COLOR_RGB2GRAY)
            min_loc = np.unravel_index(np.argmin(gray_motif), gray_motif.shape)
            # np.unravel_index returns (row, col) which is (y, x).
            current_start = (int(min_loc[1]), int(min_loc[0]))  # (x, y)
            print(f"Starting from the darkest point in motif: {current_start}")
        else:
            # Default canvas if no motif
            canvas_rgb = np.ones((256, 256, 3), dtype=np.uint8) * 255
            h, w = canvas_rgb.shape[:2]
            current_start = (w // 2, h // 2)
        
        # Store a copy of the initial RGB canvas. This is the "master" initial image.
        initial_canvas = canvas_rgb.copy()  # This is a pristine white RGB canvas
        
        # Convert images to BGR for all OpenCV operations that follow
        # The main canvas for drawing will be in BGR
        canvas = cv2.cvtColor(canvas_rgb, cv2.COLOR_RGB2BGR)
        # The motif for comparison will also be in BGR
        motif = cv2.cvtColor(motif_rgb, cv2.COLOR_RGB2BGR) if motif_rgb is not None else None
        
        return canvas, motif, current_start, initial_canvas
    
    def create_output_directory(self):
        """
        Create timestamped output directory.
        """
        base_out = "outputs"
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(base_out, timestamp)
        os.makedirs(self.out_dir, exist_ok=True)
        return self.out_dir
    
    def save_motif(self, motif):
        """
        Save motif image to output directory.
        """
        if motif is not None and self.out_dir:
            motif_path_out = os.path.join(self.out_dir, "motif.png")
            cv2.imwrite(motif_path_out, motif)
            print(f"Motif image saved as {motif_path_out}")
    
    def save_results(self, canvas, initial_canvas, distances, total_reward, current_start, converged, executed_steps):
        """
        Save all run results including images, video, and summary.
        """
        if not self.out_dir:
            return
        
        # Save initial blank canvas image
        in_path = os.path.join(self.out_dir, "initial_canvas.png")
        if not self.args.resume_from:
            cv2.imwrite(in_path, cv2.cvtColor(initial_canvas, cv2.COLOR_RGB2BGR))
        else:
            cv2.imwrite(in_path, initial_canvas)  # Already BGR
        print(f"Initial canvas image saved as {in_path}")
        
        # Save final canvas image
        final_path = os.path.join(self.out_dir, "final_canvas.png")
        cv2.imwrite(final_path, canvas)
        print(f"Final canvas image saved as {final_path}")
        
        # Create and save summary
        config_to_save = {k: str(v) if v is not None else None for k, v in vars(self.args).items()}
        summary = {'config': config_to_save}
        if self.resumed_from_path:
            summary['resumed_from'] = self.resumed_from_path
        
        total_steps_so_far = self.previous_total_steps + executed_steps
        
        if distances:
            stats = np.array(distances)
            summary['results'] = {
                'converged': converged,
                'steps_in_this_run': executed_steps,
                'total_steps_from_start': total_steps_so_far,
                'last_position': current_start,
                'total_reward_this_run': total_reward,
                'initial_distance': distances[0] if distances else None,
                'final_distance': distances[-1],
                'distance_stats_full_run': {
                    'min': float(stats.min()),
                    'max': float(stats.max()),
                    'mean': float(stats.mean()),
                    'var': float(stats.var())
                }
            }
            
            # Print info to console
            print(f"Total reward for this run after {executed_steps} steps: {total_reward:.2f}")
            print(f"Distance stats (full) - min: {stats.min():.2f}, max: {stats.max():.2f}, mean: {stats.mean():.2f}, var: {stats.var():.2f}")
            
            # Save distances list
            distances_path = os.path.join(self.out_dir, "distances.json")
            with open(distances_path, 'w') as f:
                json.dump(distances, f)
        
        summary_path = os.path.join(self.out_dir, "run_summary.json")
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=4)
        print(f"Run summary saved to {summary_path}")
