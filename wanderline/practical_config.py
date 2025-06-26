"""
Practical Configuration Class for Wanderline
============================================

Lightweight, typed configuration that solves real problems:
- Type safety with IDE autocomplete 
- Safe resume without mutations
- Auto-adjustments (n_samples, visualization)
- Minimal complexity
"""

from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any
from argparse import Namespace


@dataclass
class WanderlineConfig:
    """
    Lightweight typed configuration for Wanderline.
    
    Provides type safety, auto-adjustments, and safe resume functionality
    without the complexity of immutable systems.
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
    visualization: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        """Apply automatic adjustments and validation."""
        self._auto_select_n_samples()
        self._initialize_visualization()
        self._sync_legacy_flags()
    
    def _auto_select_n_samples(self):
        """Auto-select n_samples based on lookahead_depth if not specified."""
        if self.n_samples is None:
            n_samples_map = {1: 36, 2: 16}
            self.n_samples = n_samples_map.get(self.lookahead_depth, 12)
    
    def _initialize_visualization(self):
        """Initialize visualization settings with auto-disable logic."""
        if self.visualization is None:
            self.visualization = {'enabled': True}
        
        # Auto-disable visualization for performance modes
        if self.ultra_fast or self.headless:
            self.visualization['enabled'] = False
            if self.ultra_fast and not self.headless:
                print("Ultra-fast mode: Real-time visualization automatically disabled for maximum performance")
    
    def _sync_legacy_flags(self):
        """Sync greedy flag with agent_type for legacy compatibility."""
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
        self._validate_drawing_params()
        self._validate_reward_params()
        self._validate_agent_params()
    
    def _validate_drawing_params(self):
        """Validate drawing-related parameters."""
        if not (0.0 < self.ratio <= 1.0):
            raise ValueError(f"ratio must be > 0 and <= 1, got {self.ratio}")
        
        if self.steps < 0:
            raise ValueError(f"steps must be non-negative, got {self.steps}")
    
    def _validate_reward_params(self):
        """Validate reward-related parameters."""
        if self.reward_type == 'l2_white_penalty' and self.white_penalty is None:
            raise ValueError("white_penalty required when using l2_white_penalty")
    
    def _validate_agent_params(self):
        """Validate agent-related parameters."""
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