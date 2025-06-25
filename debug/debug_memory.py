import numpy as np
import psutil
import os

def estimate_memory_usage():
    """
    Estimate memory usage for different vectorization approaches.
    """
    # Typical image size
    h, w, c = 500, 500, 3
    canvas_size_bytes = h * w * c
    
    print(f"Canvas size: {h}×{w}×{c} = {canvas_size_bytes:,} bytes ({canvas_size_bytes/(1024*1024):.1f}MB)")
    
    # Different scenarios
    scenarios = [
        ("1-step, 36 samples", 36, 1),
        ("2-step, 12 samples", 12, 2),
        ("2-step, 16 samples", 16, 2),
        ("2-step, 20 samples", 20, 2),
        ("2-step, 24 samples", 24, 2),
    ]
    
    for name, n_samples, depth in scenarios:
        if depth == 1:
            total_canvases = n_samples
        elif depth == 2:
            total_canvases = n_samples + n_samples**2  # First level + all combinations
        
        memory_mb = (total_canvases * canvas_size_bytes) / (1024 * 1024)
        print(f"{name:20}: {total_canvases:4} canvases = {memory_mb:6.1f}MB")

def get_current_memory():
    """Get current process memory usage."""
    process = psutil.Process(os.getpid())
    memory_info = process.memory_info()
    return memory_info.rss / (1024 * 1024)  # MB

if __name__ == "__main__":
    print("Memory usage estimation for vectorized drawing:")
    print("=" * 50)
    estimate_memory_usage()
    print("\nCurrent process memory:", f"{get_current_memory():.1f}MB")
    
    # Check system memory
    memory = psutil.virtual_memory()
    print(f"System memory: {memory.total/(1024**3):.1f}GB total, {memory.available/(1024**3):.1f}GB available")
