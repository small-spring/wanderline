#!/usr/bin/env python3
"""
Memory usage analysis for vectorized agent implementations.
"""
import numpy as np
import tracemalloc
import sys
import os

# Add parent directory to path to import wanderline modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wanderline.agent import (
    choose_next_angle_vectorized, 
    choose_next_angle_fully_vectorized_2step,
    choose_next_angle_vectorized_lookahead
)


def analyze_memory_usage():
    """Analyze memory usage for different agent implementations."""
    
    print("=== Memory Usage Analysis for Vectorized Agents ===\n")
    
    # Test parameters - similar to actual usage
    canvas_sizes = [
        (50, 50, 3),     # Small test
        (256, 256, 3),   # Medium
        (500, 500, 3),   # Actual size
    ]
    
    n_samples_list = [8, 12, 16, 24, 36]
    
    for canvas_size in canvas_sizes:
        print(f"Canvas size: {canvas_size}")
        canvas = np.ones(canvas_size, dtype=np.uint8) * 255
        motif = np.zeros(canvas_size, dtype=np.uint8)
        start_pt = (canvas_size[1]//2, canvas_size[0]//2)
        length = min(canvas_size[0], canvas_size[1]) // 10
        
        single_canvas_mb = canvas.nbytes / (1024 * 1024)
        print(f"Single canvas size: {single_canvas_mb:.2f}MB")
        
        for n_samples in n_samples_list:
            print(f"\n  n_samples = {n_samples}")
            
            # 1. Test vectorized 1-step
            try:
                tracemalloc.start()
                
                result, timing = choose_next_angle_vectorized(
                    canvas, motif, start_pt, length, n_samples, verbose=True
                )
                
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()
                
                estimated_batch_mb = single_canvas_mb * n_samples
                print(f"    1-step: Peak memory: {peak/1024/1024:.1f}MB, Estimated batch: {estimated_batch_mb:.1f}MB")
                
            except Exception as e:
                print(f"    1-step: FAILED - {e}")
            
            # 2. Test fully vectorized 2-step (if reasonable size)
            total_combinations = n_samples * n_samples
            estimated_2step_mb = single_canvas_mb * total_combinations * 2  # Two batches
            
            print(f"    2-step estimated memory: {estimated_2step_mb:.1f}MB ({total_combinations} combinations)")
            
            if estimated_2step_mb < 1000:  # Only try if less than 1GB
                try:
                    tracemalloc.start()
                    
                    result, timing = choose_next_angle_fully_vectorized_2step(
                        canvas, motif, start_pt, length, n_samples, verbose=True
                    )
                    
                    current, peak = tracemalloc.get_traced_memory()
                    tracemalloc.stop()
                    
                    print(f"    2-step actual: Peak memory: {peak/1024/1024:.1f}MB")
                    
                except Exception as e:
                    print(f"    2-step: FAILED - {e}")
            else:
                print("    2-step: SKIPPED (too much memory)")
        
        print("\n" + "="*50)


def test_different_approaches():
    """Test different memory optimization approaches."""
    
    print("\n=== Testing Memory Optimization Approaches ===\n")
    
    # Use actual canvas size
    canvas_size = (500, 500, 3)
    canvas = np.ones(canvas_size, dtype=np.uint8) * 255
    motif = np.zeros(canvas_size, dtype=np.uint8)
    start_pt = (250, 250)
    length = 50
    
    print(f"Canvas size: {canvas_size}")
    single_canvas_mb = canvas.nbytes / (1024 * 1024)
    print(f"Single canvas: {single_canvas_mb:.2f}MB")
    
    # Test partial vectorization approach (current implementation)
    n_samples = 12
    print(f"\nTesting partial vectorization with n_samples={n_samples}")
    
    try:
        tracemalloc.start()
        
        result = choose_next_angle_vectorized_lookahead(
            canvas, motif, start_pt, length, n_samples, 2
        )
        
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        
        print(f"Partial vectorization: Peak memory: {peak/1024/1024:.1f}MB")
        print(f"Result: {result}")
        
    except Exception as e:
        print(f"Partial vectorization FAILED: {e}")


def estimate_system_limits():
    """Estimate system memory limits for vectorization."""
    
    print("\n=== System Memory Limits Analysis ===\n")
    
    canvas_size = (500, 500, 3)
    single_canvas_mb = np.ones(canvas_size, dtype=np.uint8).nbytes / (1024 * 1024)
    
    print(f"Single canvas (500x500x3): {single_canvas_mb:.2f}MB")
    
    memory_budgets = [100, 500, 1000, 2000]  # MB
    
    for budget_mb in memory_budgets:
        max_batch_size = int(budget_mb / single_canvas_mb)
        max_n_samples_1step = max_batch_size
        max_n_samples_2step = int(np.sqrt(max_batch_size / 2))  # Divide by 2 for two batches
        
        print(f"\nMemory budget: {budget_mb}MB")
        print(f"  Max 1-step n_samples: {max_n_samples_1step}")
        print(f"  Max 2-step n_samples: {max_n_samples_2step}")
        print(f"  2-step combinations: {max_n_samples_2step**2}")


if __name__ == "__main__":
    try:
        analyze_memory_usage()
        test_different_approaches()
        estimate_system_limits()
    except KeyboardInterrupt:
        print("\nAnalysis interrupted by user")
    except Exception as e:
        print(f"Analysis failed: {e}")
        import traceback
        traceback.print_exc()
