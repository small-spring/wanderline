"""
Practical Configuration Class Specification for Wanderline
=========================================================

A lightweight, typed configuration system that solves real problems
without over-engineering. Focused on type safety, IDE support, and
safe resume functionality.
"""

from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any
from argparse import Namespace


@dataclass
class WanderlineConfig:
    """
    Lightweight typed configuration for Wanderline.
    
    Design principles:
    - Type safety with IDE autocomplete
    - Logical grouping of related settings
    - Safe resume without mutations
    - Minimal migration from current argparse
    - No complex lifecycle management
    """
    
    # Core settings
    motif_path: Optional[str] = None
    resume_from: Optional[str] = None
    
    # Agent settings (most accessed)
    agent_type: Literal['greedy', 'transformer'] = 'greedy'
    greedy: bool = True  # Legacy compatibility
    lookahead_depth: int = 1
    n_samples: Optional[int] = None
    fast_agent: bool = False
    ultra_fast: bool = False
    
    # Drawing settings
    steps: int = 100
    duration: float = 15.0
    opacity: float = 1.0
    line_width: int = 3
    ratio: float = 0.2
    
    # Reward settings
    reward_type: Literal['l2', 'l2_white_penalty', 'l2_coverage_bonus', 'l2_exploration_bonus'] = 'l2'
    white_penalty: Optional[float] = None
    
    # Optimization settings
    memory_efficient: bool = False
    patience: int = 10
    min_delta: float = 1e-4
    verbose: bool = False
    headless: bool = False
    
    # Visualization (dict for backward compatibility)
    visualization: Dict[str, Any] = None
    
    def __post_init__(self):
        """Apply automatic adjustments and validation."""
        # Auto-select n_samples based on lookahead_depth
        if self.n_samples is None:
            if self.lookahead_depth == 1:
                self.n_samples = 36
            elif self.lookahead_depth == 2:
                self.n_samples = 16
            else:
                self.n_samples = 12
        
        # Ultra-fast mode auto-disables visualization
        if self.visualization is None:
            self.visualization = {'enabled': True}
        
        if self.ultra_fast or self.headless:
            self.visualization['enabled'] = False
            if self.ultra_fast and not self.headless:
                print("Ultra-fast mode: Real-time visualization automatically disabled for maximum performance")
        
        # Ensure greedy flag matches agent_type for legacy compatibility
        if self.agent_type == 'greedy':
            self.greedy = True
        elif self.agent_type == 'transformer':
            self.greedy = False
    
    @property
    def is_fast_mode(self) -> bool:
        """Check if any fast mode is enabled."""
        return self.fast_agent or self.ultra_fast
    
    def validate(self) -> None:
        """Validate configuration constraints."""
        if not (0.0 < self.ratio <= 1.0):
            raise ValueError(f"ratio must be > 0 and <= 1, got {self.ratio}")
        
        if self.steps < 0:
            raise ValueError(f"steps must be non-negative, got {self.steps}")
        
        if self.reward_type == 'l2_white_penalty' and self.white_penalty is None:
            raise ValueError("white_penalty required when using l2_white_penalty")
        
        if self.lookahead_depth < 1:
            raise ValueError(f"lookahead_depth must be >= 1, got {self.lookahead_depth}")
        
        if self.is_fast_mode and self.lookahead_depth != 1:
            raise ValueError("Fast modes only work with lookahead_depth=1")
    
    def create_for_resume(self, resume_data: Dict[str, Any]) -> 'WanderlineConfig':
        """Create new config for resume (safe alternative to mutation)."""
        # Create new config with overrides
        new_data = self.__dict__.copy()
        
        # Map resume data to config fields
        if 'motif_path' in resume_data:
            new_data['motif_path'] = resume_data['motif_path']
        if 'ratio' in resume_data:
            new_data['ratio'] = float(resume_data['ratio'])
        if 'opacity' in resume_data:
            new_data['opacity'] = float(resume_data['opacity'])
        if 'greedy' in resume_data:
            new_data['greedy'] = str(resume_data['greedy']).lower() == 'true'
            new_data['agent_type'] = 'greedy' if new_data['greedy'] else 'transformer'
        
        return WanderlineConfig(**new_data)
    
    @classmethod
    def from_args(cls, args: Namespace) -> 'WanderlineConfig':
        """Create config from argparse Namespace."""
        # Extract all known fields from args
        config_data = {}
        
        # Direct field mappings
        field_mappings = [
            'motif_path', 'resume_from', 'agent_type', 'greedy', 'lookahead_depth',
            'n_samples', 'fast_agent', 'ultra_fast', 'steps', 'duration', 
            'opacity', 'line_width', 'ratio', 'reward_type', 'white_penalty',
            'memory_efficient', 'patience', 'min_delta', 'verbose', 'headless'
        ]
        
        for field in field_mappings:
            if hasattr(args, field):
                config_data[field] = getattr(args, field)
        
        # Handle visualization dict
        if hasattr(args, 'visualization'):
            config_data['visualization'] = args.visualization
        
        return cls(**config_data)
    
    def to_args_namespace(self) -> Namespace:
        """Convert back to argparse Namespace for legacy compatibility."""
        return Namespace(**self.__dict__)


# Factory Functions

def create_config(json_path: Optional[str] = None, args: Optional[Namespace] = None) -> WanderlineConfig:
    """
    Create configuration from JSON and/or CLI args.
    
    Simple factory function without complex lifecycle management.
    """
    import json
    from pathlib import Path
    
    config_data = {}
    
    # Load from JSON if provided
    if json_path and Path(json_path).exists():
        with open(json_path, 'r') as f:
            json_data = json.load(f)
        
        # Map JSON structure to flat config
        config_data.update(json_data)
        
        # Handle nested structures
        if 'agent' in json_data:
            config_data.update(json_data['agent'])
        if 'drawing' in json_data:
            config_data.update(json_data['drawing'])
        if 'reward' in json_data:
            config_data.update(json_data['reward'])
        if 'optimization' in json_data:
            config_data.update(json_data['optimization'])
    
    # Override with CLI args
    if args:
        args_data = {k: v for k, v in vars(args).items() if v is not None}
        config_data.update(args_data)
    
    config = WanderlineConfig(**config_data)
    config.validate()
    return config


# Migration Helper

def migrate_getattr_calls():
    """
    Helper function showing how to migrate from getattr patterns.
    
    BEFORE:
        white_penalty = getattr(args, 'white_penalty', None)
        use_ultra_fast = getattr(self.args, 'ultra_fast', False)
    
    AFTER:
        white_penalty = config.white_penalty
        use_ultra_fast = config.ultra_fast
    """
    pass


# Usage Examples

def example_basic_usage():
    """Basic typed configuration usage."""
    from argparse import Namespace
    
    # Create from args
    args = Namespace(ultra_fast=True, steps=500, opacity=0.3)
    config = WanderlineConfig.from_args(args)
    
    # Type-safe access with IDE support
    print(f"Ultra fast: {config.ultra_fast}")      # IDE knows this is bool
    print(f"Steps: {config.steps}")                # IDE knows this is int  
    print(f"Is fast mode: {config.is_fast_mode}")  # IDE knows this is bool property
    
    # Auto-adjustments applied
    assert config.n_samples == 36  # Auto-selected for lookahead_depth=1
    assert config.visualization['enabled'] is False  # Auto-disabled for ultra_fast

def example_resume_safety():
    """Safe resume without mutations."""
    config = WanderlineConfig(motif_path="original.png", steps=500)
    
    # Safe resume - creates new config
    resume_data = {'motif_path': 'resumed.png', 'ratio': 0.3}
    resumed_config = config.create_for_resume(resume_data)
    
    # Original unchanged
    assert config.motif_path == "original.png"
    assert resumed_config.motif_path == "resumed.png"

def example_validation():
    """Configuration validation."""
    try:
        config = WanderlineConfig(ultra_fast=True, lookahead_depth=2)
        config.validate()  # Will raise ValueError
    except ValueError as e:
        print(f"Validation caught: {e}")


if __name__ == "__main__":
    print("Running configuration examples...")
    example_basic_usage()
    example_resume_safety()
    example_validation()
    print("✅ All examples passed!")