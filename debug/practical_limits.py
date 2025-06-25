#!/usr/bin/env python3
"""
Practical vectorization analysis - finding the sweet spot.
"""
import numpy as np
import time
import sys
import os

# Add parent directory to path to import wanderline modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wanderline.agent import (
    choose_next_angle_vectorized, 
    choose_next_angle_vectorized_lookahead,
    _choose_next_angle_greedy
)


def find_practical_limits():
    """Find practical limits for vectorization on real canvas sizes."""
    
    print("=== Finding Practical Vectorization Limits ===\n")
    
    # Use actual canvas size
    canvas_size = (500, 500, 3)
    canvas = np.ones(canvas_size, dtype=np.uint8) * 255
    motif = np.zeros(canvas_size, dtype=np.uint8)
    start_pt = (250, 250)
    length = 50
    
    print(f"Canvas size: {canvas_size}")
    print(f"Single canvas memory: {canvas.nbytes / (1024*1024):.2f}MB\n")
    
    # Test 1-step vectorization with different n_samples
    print("1-step vectorization:")
    for n_samples in [8, 12, 16, 24, 36, 48]:
        try:
            start_time = time.time()
            result, timing = choose_next_angle_vectorized(
                canvas, motif, start_pt, length, n_samples, verbose=False
            )
            elapsed = time.time() - start_time
            
            print(f"  n_samples={n_samples:2d}: {elapsed:.3f}s - SUCCESS")
            
        except Exception as e:
            print(f"  n_samples={n_samples:2d}: FAILED - {e}")
    
    print("\n2-step partial vectorization:")
    for n_samples in [4, 6, 8, 10, 12, 16]:
        combinations = n_samples * n_samples
        try:
            start_time = time.time()
            _ = choose_next_angle_vectorized_lookahead(
                canvas, motif, start_pt, length, n_samples, 2
            )
            elapsed = time.time() - start_time
            
            print(f"  n_samples={n_samples:2d} ({combinations:3d} comb): {elapsed:.3f}s - SUCCESS")
            
        except Exception as e:
            print(f"  n_samples={n_samples:2d} ({combinations:3d} comb): FAILED - {e}")
    
    print("\nSequential vs Vectorized comparison (n_samples=12):")
    n_samples = 12
    
    # Sequential 1-step
    start_time = time.time()
    seq_result = _choose_next_angle_greedy(canvas, motif, start_pt, length, n_samples)
    seq_time = time.time() - start_time
    
    # Vectorized 1-step
    start_time = time.time()
    vec_result, _ = choose_next_angle_vectorized(canvas, motif, start_pt, length, n_samples)
    vec_time = time.time() - start_time
    
    # Partial vectorized 2-step
    start_time = time.time()
    _ = choose_next_angle_vectorized_lookahead(canvas, motif, start_pt, length, n_samples, 2)
    vec2_time = time.time() - start_time
    
    print(f"  Sequential 1-step:    {seq_time:.3f}s")
    print(f"  Vectorized 1-step:    {vec_time:.3f}s (speedup: {seq_time/vec_time:.1f}x)")
    print(f"  Vectorized 2-step:    {vec2_time:.3f}s")
    print(f"  Results match: seq={seq_result:.3f}, vec={vec_result:.3f}")


def recommend_settings():
    """Recommend practical settings based on performance analysis."""
    
    print("\n=== Recommendations ===\n")
    
    canvas_size = (500, 500, 3)
    canvas = np.ones(canvas_size, dtype=np.uint8) * 255
    motif = np.zeros(canvas_size, dtype=np.uint8)
    start_pt = (250, 250)
    length = 50
    
    settings = [
        {"desc": "Fast 1-step", "depth": 1, "n_samples": 36, "target_time": 0.1},
        {"desc": "Balanced 2-step", "depth": 2, "n_samples": 12, "target_time": 0.5},
        {"desc": "Quality 2-step", "depth": 2, "n_samples": 16, "target_time": 2.0},
    ]
    
    for setting in settings:
        print(f"{setting['desc']} (depth={setting['depth']}, n_samples={setting['n_samples']}):")
        try:
            if setting['depth'] == 1:
                start_time = time.time()
                _ = choose_next_angle_vectorized(
                    canvas, motif, start_pt, length, setting['n_samples']
                )
                elapsed = time.time() - start_time
            else:
                start_time = time.time()
                _ = choose_next_angle_vectorized_lookahead(
                    canvas, motif, start_pt, length, setting['n_samples'], setting['depth']
                )
                elapsed = time.time() - start_time
            
            status = "✓ GOOD" if elapsed <= setting['target_time'] else "⚠ SLOW"
            print(f"  Time: {elapsed:.3f}s (target: {setting['target_time']}s) - {status}")
            
        except Exception as e:
            print(f"  FAILED: {e}")
        
        print()


if __name__ == "__main__":
    try:
        find_practical_limits()
        recommend_settings()
    except KeyboardInterrupt:
        print("\nAnalysis interrupted by user")
    except Exception as e:
        print(f"Analysis failed: {e}")
        import traceback
        traceback.print_exc()
