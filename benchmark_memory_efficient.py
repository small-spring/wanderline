#!/usr/bin/env python3
"""
Benchmark script to compare execution time between normal and memory_efficient modes.
"""

import time
import subprocess
import json
import sys
from pathlib import Path

def run_benchmark(config_override, description, steps=100):
    """Run a single benchmark with timing."""
    print(f"\n🔄 Running {description}...")
    
    # Create temporary config
    config = {
        "motif_path": "assets/sample.png",
        "ratio": 0.1,
        "steps": steps,
        "duration": 5.0,
        "opacity": 0.3,
        "line_width": 2,
        "greedy": True,
        "lookahead_depth": 1,
        "n_samples": 36,
        "verbose": False,
        "headless": True,  # Disable visualization for accurate timing
        **config_override
    }
    
    config_path = f"temp_config_{description.replace(' ', '_').lower()}.json"
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    
    # Run the test
    start_time = time.time()
    result = subprocess.run([
        "uv", "run", "python", "run_test.py", 
        "--config", config_path,
        "--headless"  # Ensure no visualization
    ], capture_output=True, text=True)
    end_time = time.time()
    
    # Clean up temp config
    Path(config_path).unlink()
    
    execution_time = end_time - start_time
    
    if result.returncode != 0:
        print(f"❌ {description} failed:")
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        return None
    
    print(f"✅ {description}: {execution_time:.2f}s")
    return execution_time

def main():
    print("🚀 Memory Efficient Mode Performance Benchmark")
    print("=" * 50)
    
    steps = 200  # Small enough for quick testing, large enough to see differences
    
    # Test configurations
    tests = [
        ({"memory_efficient": False}, "Normal Mode", steps),
        ({"memory_efficient": True}, "Memory Efficient Mode", steps),
    ]
    
    results = {}
    
    for config_override, description, test_steps in tests:
        execution_time = run_benchmark(config_override, description, test_steps)
        if execution_time is not None:
            results[description] = execution_time
    
    # Print comparison
    print("\n📊 Results Summary")
    print("=" * 30)
    
    if len(results) == 2:
        normal_time = results["Normal Mode"]
        memory_efficient_time = results["Memory Efficient Mode"]
        
        print(f"Normal Mode:          {normal_time:.2f}s")
        print(f"Memory Efficient:     {memory_efficient_time:.2f}s")
        
        if memory_efficient_time < normal_time:
            speedup = normal_time / memory_efficient_time
            print(f"🚀 Memory efficient is {speedup:.2f}x FASTER")
        else:
            slowdown = memory_efficient_time / normal_time
            print(f"🐌 Memory efficient is {slowdown:.2f}x SLOWER")
        
        time_diff = abs(memory_efficient_time - normal_time)
        print(f"Time difference: {time_diff:.2f}s")
    
    print(f"\n🔬 Test parameters: {steps} steps, headless mode, greedy algorithm")

if __name__ == "__main__":
    main()