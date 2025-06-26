#!/usr/bin/env python3
"""
Practical Config Integration Example
===================================

Shows how to integrate the practical config class with the existing
Wanderline codebase, demonstrating gradual migration from argparse.
"""

from wanderline.practical_config import WanderlineConfig, create_config
from wanderline.config_manager import load_config_and_parse_args
from argparse import Namespace


def example_current_vs_new_approach():
    """Compare current argparse approach vs new typed config."""
    print("=== Current vs New Approach ===")
    
    # Current approach (what we have now)
    args = load_config_and_parse_args()
    
    # New approach (what we're building)
    config = WanderlineConfig.from_args(args)
    
    print("BEFORE (current argparse):")
    print(f"  Ultra-fast: {getattr(args, 'ultra_fast', False)}")  # ❌ No type safety
    print(f"  Steps: {getattr(args, 'steps', 100)}")             # ❌ No IDE support
    
    print("AFTER (typed config):")
    print(f"  Ultra-fast: {config.ultra_fast}")  # ✅ Type-safe, IDE autocomplete
    print(f"  Steps: {config.steps}")            # ✅ Full IDE support
    print(f"  Auto n_samples: {config.n_samples}")  # ✅ Auto-selected value
    print()


def example_migration_strategy():
    """Show how to gradually migrate existing code."""
    print("=== Migration Strategy ===")
    
    # Step 1: Create config alongside existing args
    args = Namespace(ultra_fast=True, steps=500, opacity=0.3)
    config = WanderlineConfig.from_args(args)
    
    print("Phase 1 - Side-by-side:")
    print(f"  Legacy: getattr(args, 'ultra_fast', False) = {getattr(args, 'ultra_fast', False)}")
    print(f"  New: config.ultra_fast = {config.ultra_fast}")
    
    # Step 2: Replace one module at a time
    def drawing_engine_new_style(config: WanderlineConfig):
        """Example of how DrawingEngine would use new config."""
        # BEFORE: if getattr(self.args, 'ultra_fast', False):
        # AFTER:
        if config.ultra_fast:
            print("  ✅ Using ultra-fast mode (type-safe)")
        
        # BEFORE: stroke_length = int(min(w, h) * self.args.ratio)
        # AFTER:
        stroke_length = int(min(800, 600) * config.ratio)
        print(f"  ✅ Stroke length: {stroke_length}")
        
        # BEFORE: for i in range(self.args.steps):
        # AFTER:
        print(f"  ✅ Running {config.steps} steps")
    
    drawing_engine_new_style(config)
    print()


def example_safe_resume():
    """Demonstrate safe resume functionality."""
    print("=== Safe Resume Example ===")
    
    # Original run config
    original_config = WanderlineConfig(
        motif_path="original.png",
        steps=500,
        opacity=0.5,
        agent_type='greedy'
    )
    
    print("Original config:")
    print(f"  Motif: {original_config.motif_path}")
    print(f"  Steps: {original_config.steps}")
    print(f"  Agent: {original_config.agent_type}")
    
    # Resume data from saved state
    resume_data = {
        'motif_path': 'resumed.png',
        'ratio': '0.3',  # String from JSON
        'greedy': 'false'  # String from JSON
    }
    
    # Safe resume - creates new config
    resumed_config = original_config.create_for_resume(resume_data)
    
    print("\nResumed config:")
    print(f"  Motif: {resumed_config.motif_path}")  # Updated
    print(f"  Ratio: {resumed_config.ratio}")        # Updated (converted from string)
    print(f"  Steps: {resumed_config.steps}")        # Preserved from original
    print(f"  Agent: {resumed_config.agent_type}")   # Updated based on greedy flag
    
    print("\n✅ Original config unchanged - no mutations!")
    print(f"  Original motif still: {original_config.motif_path}")
    print()


def example_validation_benefits():
    """Show validation benefits."""
    print("=== Validation Benefits ===")
    
    try:
        # This catches errors early
        config = WanderlineConfig(
            ultra_fast=True,
            lookahead_depth=2,  # Invalid combination
            steps=500
        )
        config.validate()
    except ValueError as e:
        print(f"✅ Validation caught error: {e}")
    
    try:
        # This also catches configuration errors
        config = WanderlineConfig(
            reward_type='l2_white_penalty'
            # Missing white_penalty value
        )
        config.validate()
    except ValueError as e:
        print(f"✅ Validation caught error: {e}")
    
    print("✅ Early error detection prevents runtime issues")
    print()


def example_type_safety_benefits():
    """Demonstrate type safety benefits."""
    print("=== Type Safety Benefits ===")
    
    config = WanderlineConfig(ultra_fast=True, steps=500)
    
    # These have full IDE support:
    print(f"Ultra-fast enabled: {config.ultra_fast}")      # IDE knows: bool
    print(f"Steps to run: {config.steps}")                 # IDE knows: int  
    print(f"Stroke opacity: {config.opacity}")             # IDE knows: float
    print(f"Is any fast mode: {config.is_fast_mode}")      # IDE knows: bool
    print(f"Agent type: {config.agent_type}")              # IDE knows: Literal
    
    # Typos caught at development time:
    # config.ulta_fast  # ❌ IDE would highlight this as error
    # config.step       # ❌ IDE would highlight this as error
    
    print("✅ Full IDE autocomplete and typo detection")
    print()


def example_backward_compatibility():
    """Show backward compatibility bridge."""
    print("=== Backward Compatibility ===")
    
    # New config
    config = WanderlineConfig(ultra_fast=True, steps=500)
    
    # Convert back to argparse for legacy code
    legacy_args = config.to_args_namespace()
    
    print("Config can be converted back to argparse:")
    print(f"  legacy_args.ultra_fast = {legacy_args.ultra_fast}")
    print(f"  legacy_args.steps = {legacy_args.steps}")
    
    # This allows gradual migration
    def legacy_function(args):
        """Old function that expects argparse Namespace."""
        return f"Processing {args.steps} steps with ultra_fast={args.ultra_fast}"
    
    result = legacy_function(legacy_args)
    print(f"  Legacy function result: {result}")
    print("✅ Smooth migration path for existing code")
    print()


if __name__ == "__main__":
    print("Practical Config Integration Examples")
    print("=" * 50)
    print()
    
    # Skip the first example that requires actual argparse setup
    # example_current_vs_new_approach()
    
    example_migration_strategy()
    example_safe_resume()
    example_validation_benefits()
    example_type_safety_benefits()
    example_backward_compatibility()
    
    print("🎯 Integration examples completed!")
    print("\nNext steps:")
    print("1. Integrate with run_test.py")
    print("2. Migrate DrawingEngine to use typed config")
    print("3. Migrate RunManager to use safe resume")
    print("4. Gradually replace getattr() calls")