"""
Wanderline Constants - Centralized configuration for hardcoded values.

This file consolidates magic numbers and hardcoded values found throughout
the codebase to make them configurable and well-documented.
"""

# === CANVAS AND DRAWING CONSTANTS ===

# Default stroke length as ratio of canvas min dimension
DEFAULT_STROKE_LENGTH_RATIO = 0.4

# White color value for canvas initialization and detection
WHITE_COLOR_VALUE = 255

# Default canvas size when no motif is provided (width, height)
DEFAULT_CANVAS_SIZE = (256, 256)

# Black stroke color (RGB)
STROKE_COLOR = (0, 0, 0)

# Opacity threshold for fully opaque drawing
OPACITY_THRESHOLD = 1.0

# === ALGORITHM CONSTANTS ===

# Default number of angle samples for greedy search
DEFAULT_N_SAMPLES = 36

# Performance mode sample counts
PERFORMANCE_SAMPLES = {
    'standard': 36,
    'fast': 24,
    'ultra_fast': 16,
    'headless': 36
}

# Progressive refinement constants
PROGRESSIVE_REFINEMENT_THRESHOLD = 16  # Minimum samples to enable progressive refinement
COARSE_SEARCH_ANGLES = 8  # Number of cardinal/diagonal angles for initial search
TOP_REGIONS_COUNT = 2  # Number of best regions to refine
REGION_WIDTH_RADIANS = 3.14159 / 8  # π/8 radians = 22.5 degrees

# Multi-step lookahead warning threshold
LOOKAHEAD_PERFORMANCE_WARNING_THRESHOLD = 24

# === VIDEO AND RECORDING CONSTANTS ===

# Maximum FPS to prevent excessive frame rates
MAX_VIDEO_FPS = 60

# Default video FPS
DEFAULT_VIDEO_FPS = 5

# Progress reporting interval for video reconstruction
RECONSTRUCTION_PROGRESS_INTERVAL = 1000

# === TRANSFORMER AGENT CONSTANTS ===

# Feature dimensions for transformer model
TRANSFORMER_FEATURE_DIM = 128
TRANSFORMER_CONV_CHANNELS = [32, 64]  # Conv layer channel progression
TRANSFORMER_FC_LAYERS = [256, 128]    # Fully connected layer sizes

# Action history length
ACTION_HISTORY_LENGTH = 10

# Neural network layer dimensions and parameters
TRANSFORMER_CONV_KERNEL_SIZE = 3
TRANSFORMER_CONV_PADDING = 1

# === EARLY STOPPING CONSTANTS ===

# Default patience for early stopping
DEFAULT_PATIENCE = 10

# Default minimum delta for improvement detection
DEFAULT_MIN_DELTA = 1e-4

# === PERFORMANCE OPTIMIZATION CONSTANTS ===

# Fast agent constants
FAST_AGENT_MIN_SAMPLES = 12
FAST_AGENT_MAX_SAMPLES = 36
FAST_AGENT_IMPROVEMENT_THRESHOLD = 0.8
FAST_AGENT_HISTORY_LENGTH = 20
FAST_AGENT_RECENT_REWARDS_LENGTH = 5
FAST_AGENT_ANGLE_HISTORY_LENGTH = 10

# Early termination threshold for fast agent
FAST_AGENT_EARLY_TERMINATION_SAMPLES = 16

# === VISUALIZATION CONSTANTS ===

# ASCII visualizer dimensions
ASCII_VISUALIZER_WIDTH = 60
ASCII_VISUALIZER_HEIGHT = 30

# OpenCV window update constants
OPENCV_QUIT_KEYS = [ord('q'), 27]  # 'q' or ESC
OPENCV_SAVE_KEY = ord('s')

# Realtime visualizer constants
REALTIME_STEP_HISTORY_LIMIT = 100  # Maximum step times to keep for averaging
REALTIME_Y_OFFSET = 30  # Y offset for text display

# === MEMORY OPTIMIZATION CONSTANTS ===

# Thresholds for memory-efficient mode recommendations
MEMORY_EFFICIENT_RECOMMENDED_STEPS = 1000

# JSON formatting
JSON_INDENT = 2  # For stroke data
JSON_SUMMARY_INDENT = 4  # For run summaries

# === MATHEMATICAL CONSTANTS ===

# Common mathematical values to avoid recalculation
PI = 3.14159265359
TWO_PI = 2 * PI
PI_OVER_4 = PI / 4
PI_OVER_2 = PI / 2

# === FILE AND DIRECTORY CONSTANTS ===

# Default output directory structure
OUTPUT_DIR_PREFIX = "outputs"
MOTIF_FILENAME = "motif.png"
FINAL_CANVAS_FILENAME = "final_canvas.png"
INITIAL_CANVAS_FILENAME = "initial_canvas.png"
DRAWING_VIDEO_FILENAME = "drawing.mp4"
DISTANCE_CURVE_FILENAME = "distance_curve.png"
RUN_SUMMARY_FILENAME = "run_summary.json"
STROKE_DATA_FILENAME = "stroke_data.json"

# === VALIDATION CONSTANTS ===

# Parameter validation ranges
MIN_RATIO = 0.0  # Exclusive minimum for stroke ratio
MAX_RATIO = 1.0  # Inclusive maximum for stroke ratio
MIN_STEPS = 0    # Minimum number of steps