#!/usr/bin/env python3
"""
Example usage of the immutable configuration system.
Demonstrates the proper lifecycle and freeze() method.
"""

from wanderline.immutable_config import ConfigBuilder, create_config_from_sources
from argparse import Namespace
import json


def example_basic_usage():
    """Basic configuration creation with freeze()."""
    print("=== Basic Usage ===")
    
    # Phase 1: Construction (mutable)
    config = (ConfigBuilder()
              .motif_path("assets/sample.png")
              .agent(ultra_fast=True, lookahead_depth=1)
              .drawing(steps=500, opacity=0.3)
              .optimization(memory_efficient=True)
              .freeze())  # Phase 2: Becomes immutable
    
    # Phase 3: Runtime (read-only)
    print(f"Agent type: {config.agent.type}")
    print(f"Ultra-fast: {config.agent.ultra_fast}")
    print(f"Steps: {config.drawing.steps}")
    print(f"Visualization enabled: {config.visualization.enabled}")  # False due to ultra_fast
    print(f"Auto-selected n_samples: {config.agent.n_samples}")       # 36 for lookahead_depth=1
    print()


def example_json_and_cli():
    """Configuration from JSON file + CLI arguments."""
    print("=== JSON + CLI Override ===")
    
    # Simulate CLI arguments
    args = Namespace(
        ultra_fast=True,      # Override JSON
        steps=1000,           # Override JSON  
        headless=True         # Additional CLI flag
    )
    
    # Create config from sources (would use actual JSON file in practice)
    config = (ConfigBuilder()
              .motif_path("assets/default.png")     # From JSON
              .drawing(steps=500, opacity=0.5)      # From JSON
              .from_args(args)                      # CLI overrides
              .freeze())
    
    print(f"Steps: {config.drawing.steps}")                    # 1000 (CLI override)
    print(f"Ultra-fast: {config.agent.ultra_fast}")           # True (CLI)
    print(f"Headless: {config.optimization.headless}")        # True (CLI)
    print(f"Visualization: {config.visualization.enabled}")   # False (headless + ultra_fast)
    print()


def example_resume_functionality():
    """Resume functionality with safe config modification."""
    print("=== Resume Functionality ===")
    
    # Original config
    config = (ConfigBuilder()
              .motif_path("original.png")
              .drawing(steps=500, opacity=0.5)
              .agent(type='greedy')
              .freeze())
    
    print("Original config:")
    print(f"  Motif: {config.motif_path}")
    print(f"  Steps: {config.drawing.steps}")
    print(f"  Opacity: {config.drawing.opacity}")
    
    # Resume with different settings (safe way to "modify")
    resume_data = {
        'motif_path': 'resumed.png',
        'steps': 750,
        'opacity': 0.3,
        'greedy': True
    }
    
    resumed_config = config.for_resume(resume_data)
    
    print("\nResumed config:")
    print(f"  Motif: {resumed_config.motif_path}")
    print(f"  Steps: {resumed_config.drawing.steps}")
    print(f"  Opacity: {resumed_config.drawing.opacity}")
    
    print("\nOriginal config unchanged:")
    print(f"  Motif: {config.motif_path}")
    print(f"  Steps: {config.drawing.steps}")
    print()


def example_validation_errors():
    """Configuration validation examples."""
    print("=== Validation Examples ===")
    
    try:
        # This will fail: ultra_fast requires lookahead_depth=1
        config = (ConfigBuilder()
                  .agent(ultra_fast=True, lookahead_depth=2)
                  .freeze())
    except ValueError as e:
        print(f"Validation error: {e}")
    
    try:
        # This will fail: white_penalty required for l2_white_penalty
        config = (ConfigBuilder()
                  .reward(type='l2_white_penalty')
                  .freeze())
    except ValueError as e:
        print(f"Validation error: {e}")
    
    try:
        # This will fail: negative steps
        config = (ConfigBuilder()
                  .drawing(steps=-1)
                  .freeze())
    except ValueError as e:
        print(f"Validation error: {e}")
    
    print()


def example_immutability():
    """Demonstrate immutability guarantees."""
    print("=== Immutability Guarantees ===")
    
    builder = ConfigBuilder()
    builder.agent(type='greedy')
    
    print("Before freeze - mutations allowed:")
    builder.agent(ultra_fast=True)  # ✅ Works
    print("  Added ultra_fast=True")
    
    config = builder.freeze()
    print("After freeze - config is immutable:")
    
    try:
        builder.agent(type='transformer')  # ❌ Should fail
    except Exception as e:
        print(f"  Builder mutation failed: {type(e).__name__}")
    
    try:
        config.drawing.steps = 1000  # ❌ Should fail
    except Exception as e:
        print(f"  Config mutation failed: {type(e).__name__}")
    
    print("  ✅ Immutability enforced")
    print()


def example_type_safety():
    """Demonstrate type safety and IDE support."""
    print("=== Type Safety ===")
    
    config = (ConfigBuilder()
              .agent(ultra_fast=True)
              .drawing(steps=500)
              .freeze())
    
    # These have full IDE support with autocomplete
    print(f"Agent ultra_fast: {config.agent.ultra_fast}")        # IDE knows this is bool
    print(f"Drawing steps: {config.drawing.steps}")              # IDE knows this is int
    print(f"Reward type: {config.reward.type}")                  # IDE knows this is Literal
    print(f"Is fast mode: {config.agent.is_fast_mode}")          # IDE knows this is bool property
    print()


def example_serialization():
    """Configuration serialization example."""
    print("=== Serialization ===")
    
    config = (ConfigBuilder()
              .motif_path("test.png")
              .agent(ultra_fast=True)
              .drawing(steps=500, opacity=0.3)
              .reward(type='l2_white_penalty', white_penalty=0.1)
              .freeze())
    
    # Convert to dictionary for saving
    config_dict = config.to_dict()
    json_str = json.dumps(config_dict, indent=2)
    
    print("Serialized config:")
    print(json_str[:200] + "..." if len(json_str) > 200 else json_str)
    print()


if __name__ == "__main__":
    print("Immutable Configuration System Examples")
    print("=" * 50)
    print()
    
    example_basic_usage()
    example_json_and_cli()
    example_resume_functionality()
    example_validation_errors()
    example_immutability()
    example_type_safety()
    example_serialization()
    
    print("All examples completed successfully! ✅")