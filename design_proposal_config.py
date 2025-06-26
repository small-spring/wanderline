"""
Configuration Architecture Design Proposal
==========================================

This demonstrates a better configuration system using typed classes,
dependency injection, and clear separation of concerns.
"""

from dataclasses import dataclass, field
from typing import Optional, Literal, Dict, Any
from pathlib import Path
import json


@dataclass
class AgentConfig:
    """Configuration for drawing agents."""
    type: Literal['greedy', 'transformer'] = 'greedy'
    lookahead_depth: int = 1
    n_samples: Optional[int] = None  # Auto-selected if None
    fast_mode: bool = False
    ultra_fast: bool = False
    
    def __post_init__(self):
        """Validate agent configuration."""
        if self.lookahead_depth < 1:
            raise ValueError(f"lookahead_depth must be >= 1, got {self.lookahead_depth}")
        
        if self.n_samples is not None and self.n_samples <= 0:
            raise ValueError(f"n_samples must be > 0, got {self.n_samples}")
        
        # Auto-select n_samples based on lookahead_depth
        if self.n_samples is None:
            if self.lookahead_depth == 1:
                self.n_samples = 36
            elif self.lookahead_depth == 2:
                self.n_samples = 16
            else:
                self.n_samples = 12
    
    @property
    def is_fast_mode(self) -> bool:
        """Check if any fast mode is enabled."""
        return self.fast_mode or self.ultra_fast


@dataclass
class VisualizationConfig:
    """Configuration for real-time visualization."""
    enabled: bool = True
    mode: Literal['opencv', 'snapshots', 'both'] = 'opencv'
    update_frequency: int = 10
    window_size: tuple[int, int] = (800, 600)
    save_snapshots: bool = False
    snapshot_interval: int = 100
    
    def __post_init__(self):
        """Validate visualization configuration."""
        if self.update_frequency <= 0:
            raise ValueError(f"update_frequency must be > 0, got {self.update_frequency}")
        
        if self.snapshot_interval <= 0:
            raise ValueError(f"snapshot_interval must be > 0, got {self.snapshot_interval}")


@dataclass
class RewardConfig:
    """Configuration for reward functions."""
    type: Literal['l2', 'l2_white_penalty', 'l2_coverage_bonus'] = 'l2'
    white_penalty: Optional[float] = None
    
    def __post_init__(self):
        """Validate reward configuration."""
        if self.type == 'l2_white_penalty' and self.white_penalty is None:
            raise ValueError("white_penalty must be set when using l2_white_penalty")
        
        if self.white_penalty is not None and (self.white_penalty < 0 or self.white_penalty > 1):
            raise ValueError(f"white_penalty must be between 0 and 1, got {self.white_penalty}")


@dataclass
class DrawingConfig:
    """Configuration for drawing parameters."""
    steps: int = 100
    duration: float = 15.0
    opacity: float = 1.0
    line_width: int = 3
    ratio: float = 0.2  # Stroke length ratio
    
    def __post_init__(self):
        """Validate drawing configuration."""
        if self.steps < 0:
            raise ValueError(f"steps must be non-negative, got {self.steps}")
        
        if not (0.0 < self.ratio <= 1.0):
            raise ValueError(f"ratio must be > 0 and <= 1, got {self.ratio}")
        
        if not (0.0 <= self.opacity <= 1.0):
            raise ValueError(f"opacity must be between 0 and 1, got {self.opacity}")


@dataclass
class OptimizationConfig:
    """Configuration for optimization and performance."""
    memory_efficient: bool = False
    patience: int = 10
    min_delta: float = 1e-4
    verbose: bool = False
    headless: bool = False
    
    def __post_init__(self):
        """Validate optimization configuration."""
        if self.patience < 0:
            raise ValueError(f"patience must be non-negative, got {self.patience}")
        
        if self.min_delta < 0:
            raise ValueError(f"min_delta must be non-negative, got {self.min_delta}")


@dataclass
class WanderlineConfig:
    """Main configuration container for Wanderline system."""
    motif_path: Optional[str] = None
    resume_from: Optional[str] = None
    
    # Nested configurations
    agent: AgentConfig = field(default_factory=AgentConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    drawing: DrawingConfig = field(default_factory=DrawingConfig)
    optimization: OptimizationConfig = field(default_factory=OptimizationConfig)
    
    def __post_init__(self):
        """Apply cross-config validation and optimization."""
        # Ultra-fast mode automatically disables visualization
        if self.agent.ultra_fast and not self.optimization.headless:
            self.visualization.enabled = False
            print("Ultra-fast mode: Real-time visualization automatically disabled")
        
        # Headless mode disables visualization
        if self.optimization.headless:
            self.visualization.enabled = False
        
        # Fast modes require lookahead_depth = 1
        if self.agent.is_fast_mode and self.agent.lookahead_depth != 1:
            raise ValueError("Fast modes only work with lookahead_depth=1")
    
    @classmethod
    def from_json(cls, config_path: str) -> 'WanderlineConfig':
        """Load configuration from JSON file."""
        with open(config_path, 'r') as f:
            data = json.load(f)
        
        return cls.from_dict(data)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'WanderlineConfig':
        """Create configuration from dictionary."""
        # Extract nested configs
        agent_data = data.pop('agent', {})
        viz_data = data.pop('visualization', {})
        reward_data = data.pop('reward', {})
        drawing_data = data.pop('drawing', {})
        opt_data = data.pop('optimization', {})
        
        return cls(
            agent=AgentConfig(**agent_data),
            visualization=VisualizationConfig(**viz_data),
            reward=RewardConfig(**reward_data),
            drawing=DrawingConfig(**drawing_data),
            optimization=OptimizationConfig(**opt_data),
            **data  # Main-level configs
        )
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary for serialization."""
        return {
            'motif_path': self.motif_path,
            'resume_from': self.resume_from,
            'agent': self.agent.__dict__,
            'visualization': self.visualization.__dict__,
            'reward': self.reward.__dict__,
            'drawing': self.drawing.__dict__,
            'optimization': self.optimization.__dict__,
        }


# Usage Examples:

class DrawingEngineNew:
    """Example of how DrawingEngine would use the new config system."""
    
    def __init__(self, 
                 agent_config: AgentConfig,
                 drawing_config: DrawingConfig,
                 reward_config: RewardConfig,
                 viz_config: VisualizationConfig):
        """Constructor with dependency injection - only gets what it needs."""
        self.agent_config = agent_config
        self.drawing_config = drawing_config
        self.reward_config = reward_config
        self.viz_config = viz_config
    
    def setup_agent(self):
        """Type-safe configuration access with IDE support."""
        if self.agent_config.type == 'transformer':
            # IDE knows agent_config.type is Literal['greedy', 'transformer']
            return self._setup_transformer_agent()
        else:
            return self._setup_greedy_agent()
    
    def _setup_greedy_agent(self):
        """Example of clean configuration usage."""
        if self.agent_config.ultra_fast:
            # Clear, type-safe access
            return self._create_ultra_fast_agent(
                samples=self.agent_config.n_samples,
                lookahead=self.agent_config.lookahead_depth
            )
        return self._create_standard_agent()


class RunManagerNew:
    """Example of how RunManager would use the new config system."""
    
    def __init__(self, 
                 drawing_config: DrawingConfig,
                 optimization_config: OptimizationConfig,
                 motif_path: Optional[str] = None,
                 resume_from: Optional[str] = None):
        """Only receives the config it actually needs."""
        self.drawing_config = drawing_config
        self.optimization_config = optimization_config
        self.motif_path = motif_path
        self.resume_from = resume_from
    
    def setup_run(self):
        """Clean configuration access."""
        if self.resume_from:
            return self._resume_run()
        
        return self._create_new_run(
            steps=self.drawing_config.steps,
            opacity=self.drawing_config.opacity
        )


# Configuration Factory
def create_config_from_args_and_json(args, json_path: Optional[str] = None) -> WanderlineConfig:
    """Bridge function to create new config from current argparse system."""
    
    # Load base config from JSON if provided
    if json_path and Path(json_path).exists():
        config = WanderlineConfig.from_json(json_path)
    else:
        config = WanderlineConfig()
    
    # Override with CLI arguments
    if hasattr(args, 'motif_path') and args.motif_path:
        config.motif_path = args.motif_path
    
    if hasattr(args, 'steps'):
        config.drawing.steps = args.steps
    
    if hasattr(args, 'ultra_fast'):
        config.agent.ultra_fast = args.ultra_fast
    
    # ... etc for all CLI args
    
    return config


if __name__ == "__main__":
    # Example usage
    config = WanderlineConfig(
        motif_path="assets/sample.png",
        agent=AgentConfig(type='greedy', ultra_fast=True),
        drawing=DrawingConfig(steps=500, opacity=0.3),
        optimization=OptimizationConfig(memory_efficient=True)
    )
    
    # Type-safe access with IDE support
    print(f"Agent type: {config.agent.type}")
    print(f"Is fast mode: {config.agent.is_fast_mode}")
    print(f"Visualization enabled: {config.visualization.enabled}")