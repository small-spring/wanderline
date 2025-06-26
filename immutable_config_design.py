"""
Immutable Configuration Design with Proper Lifecycle Management
==============================================================

This design addresses configuration mutability timing by creating
an immutable configuration system with well-defined lifecycle phases.
"""

from dataclasses import dataclass, field, replace
from typing import Optional, Literal, Dict, Any, Protocol
from pathlib import Path
import json
from enum import Enum


class ConfigPhase(Enum):
    """Configuration lifecycle phases."""
    CONSTRUCTION = "construction"  # Building config from sources
    VALIDATION = "validation"     # Cross-validation and constraints
    FROZEN = "frozen"            # Immutable, ready for use
    RUNTIME = "runtime"          # Active execution (read-only)


class ImmutableConfigError(Exception):
    """Raised when attempting to modify a frozen configuration."""
    pass


@dataclass(frozen=True)
class AgentConfig:
    """Immutable agent configuration."""
    type: Literal['greedy', 'transformer'] = 'greedy'
    lookahead_depth: int = 1
    n_samples: Optional[int] = None
    fast_mode: bool = False
    ultra_fast: bool = False
    
    def __post_init__(self):
        """Validate configuration during construction."""
        if self.lookahead_depth < 1:
            raise ValueError(f"lookahead_depth must be >= 1, got {self.lookahead_depth}")
        
        if self.n_samples is not None and self.n_samples <= 0:
            raise ValueError(f"n_samples must be > 0, got {self.n_samples}")
    
    def with_auto_samples(self) -> 'AgentConfig':
        """Return new config with auto-selected n_samples."""
        if self.n_samples is not None:
            return self
        
        if self.lookahead_depth == 1:
            auto_samples = 36
        elif self.lookahead_depth == 2:
            auto_samples = 16
        else:
            auto_samples = 12
        
        return replace(self, n_samples=auto_samples)
    
    @property
    def is_fast_mode(self) -> bool:
        """Check if any fast mode is enabled."""
        return self.fast_mode or self.ultra_fast


@dataclass(frozen=True)
class VisualizationConfig:
    """Immutable visualization configuration."""
    enabled: bool = True
    mode: Literal['opencv', 'snapshots', 'both'] = 'opencv'
    update_frequency: int = 10
    window_size: tuple[int, int] = (800, 600)
    save_snapshots: bool = False
    snapshot_interval: int = 100
    
    def __post_init__(self):
        if self.update_frequency <= 0:
            raise ValueError(f"update_frequency must be > 0")
        if self.snapshot_interval <= 0:
            raise ValueError(f"snapshot_interval must be > 0")


@dataclass(frozen=True)
class RewardConfig:
    """Immutable reward configuration."""
    type: Literal['l2', 'l2_white_penalty'] = 'l2'
    white_penalty: Optional[float] = None
    
    def __post_init__(self):
        if self.type == 'l2_white_penalty' and self.white_penalty is None:
            raise ValueError("white_penalty required for l2_white_penalty")
        if self.white_penalty is not None and not (0 <= self.white_penalty <= 1):
            raise ValueError(f"white_penalty must be 0-1, got {self.white_penalty}")


@dataclass(frozen=True)
class DrawingConfig:
    """Immutable drawing configuration."""
    steps: int = 100
    duration: float = 15.0
    opacity: float = 1.0
    line_width: int = 3
    ratio: float = 0.2
    
    def __post_init__(self):
        if self.steps < 0:
            raise ValueError(f"steps must be non-negative")
        if not (0.0 < self.ratio <= 1.0):
            raise ValueError(f"ratio must be 0-1")
        if not (0.0 <= self.opacity <= 1.0):
            raise ValueError(f"opacity must be 0-1")


@dataclass(frozen=True)
class OptimizationConfig:
    """Immutable optimization configuration."""
    memory_efficient: bool = False
    patience: int = 10
    min_delta: float = 1e-4
    verbose: bool = False
    headless: bool = False
    
    def __post_init__(self):
        if self.patience < 0:
            raise ValueError(f"patience must be non-negative")
        if self.min_delta < 0:
            raise ValueError(f"min_delta must be non-negative")


class ConfigBuilder:
    """Mutable builder for creating immutable configurations."""
    
    def __init__(self):
        self._phase = ConfigPhase.CONSTRUCTION
        self._motif_path: Optional[str] = None
        self._resume_from: Optional[str] = None
        self._agent_data: Dict[str, Any] = {}
        self._viz_data: Dict[str, Any] = {}
        self._reward_data: Dict[str, Any] = {}
        self._drawing_data: Dict[str, Any] = {}
        self._opt_data: Dict[str, Any] = {}
    
    def _check_mutable(self):
        """Ensure configuration is still mutable."""
        if self._phase != ConfigPhase.CONSTRUCTION:
            raise ImmutableConfigError(f"Cannot modify config in {self._phase.value} phase")
    
    def motif_path(self, path: Optional[str]) -> 'ConfigBuilder':
        """Set motif path."""
        self._check_mutable()
        self._motif_path = path
        return self
    
    def resume_from(self, path: Optional[str]) -> 'ConfigBuilder':
        """Set resume path."""
        self._check_mutable()
        self._resume_from = path
        return self
    
    def agent(self, **kwargs) -> 'ConfigBuilder':
        """Set agent configuration."""
        self._check_mutable()
        self._agent_data.update(kwargs)
        return self
    
    def visualization(self, **kwargs) -> 'ConfigBuilder':
        """Set visualization configuration."""
        self._check_mutable()
        self._viz_data.update(kwargs)
        return self
    
    def reward(self, **kwargs) -> 'ConfigBuilder':
        """Set reward configuration."""
        self._check_mutable()
        self._reward_data.update(kwargs)
        return self
    
    def drawing(self, **kwargs) -> 'ConfigBuilder':
        """Set drawing configuration."""
        self._check_mutable()
        self._drawing_data.update(kwargs)
        return self
    
    def optimization(self, **kwargs) -> 'ConfigBuilder':
        """Set optimization configuration."""
        self._check_mutable()
        self._opt_data.update(kwargs)
        return self
    
    def from_json(self, config_path: str) -> 'ConfigBuilder':
        """Load from JSON file."""
        self._check_mutable()
        with open(config_path, 'r') as f:
            data = json.load(f)
        
        self._motif_path = data.get('motif_path')
        self._resume_from = data.get('resume_from')
        self._agent_data.update(data.get('agent', {}))
        self._viz_data.update(data.get('visualization', {}))
        self._reward_data.update(data.get('reward', {}))
        self._drawing_data.update(data.get('drawing', {}))
        self._opt_data.update(data.get('optimization', {}))
        return self
    
    def from_args(self, args) -> 'ConfigBuilder':
        """Load from argparse Namespace."""
        self._check_mutable()
        
        # Map args to config sections
        if hasattr(args, 'motif_path'):
            self._motif_path = args.motif_path
        if hasattr(args, 'resume_from'):
            self._resume_from = args.resume_from
            
        # Agent config
        agent_fields = ['type', 'lookahead_depth', 'n_samples', 'fast_mode', 'ultra_fast']
        for field in agent_fields:
            if hasattr(args, field):
                self._agent_data[field] = getattr(args, field)
        
        # Map CLI flags to config
        if hasattr(args, 'ultra_fast'):
            self._agent_data['ultra_fast'] = args.ultra_fast
        if hasattr(args, 'fast_agent'):
            self._agent_data['fast_mode'] = args.fast_agent
            
        # Continue for other sections...
        return self
    
    def build(self) -> 'WanderlineConfig':
        """Build immutable configuration."""
        if self._phase != ConfigPhase.CONSTRUCTION:
            raise ImmutableConfigError(f"Config already built in {self._phase.value} phase")
        
        self._phase = ConfigPhase.VALIDATION
        
        # Create individual configs
        agent_config = AgentConfig(**self._agent_data).with_auto_samples()
        viz_config = VisualizationConfig(**self._viz_data)
        reward_config = RewardConfig(**self._reward_data)
        drawing_config = DrawingConfig(**self._drawing_data)
        opt_config = OptimizationConfig(**self._opt_data)
        
        # Build final config
        config = WanderlineConfig(
            motif_path=self._motif_path,
            resume_from=self._resume_from,
            agent=agent_config,
            visualization=viz_config,
            reward=reward_config,
            drawing=drawing_config,
            optimization=opt_config
        )
        
        self._phase = ConfigPhase.FROZEN
        return config


@dataclass(frozen=True)
class WanderlineConfig:
    """Immutable main configuration."""
    motif_path: Optional[str] = None
    resume_from: Optional[str] = None
    agent: AgentConfig = field(default_factory=AgentConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    drawing: DrawingConfig = field(default_factory=DrawingConfig)
    optimization: OptimizationConfig = field(default_factory=OptimizationConfig)
    _phase: ConfigPhase = field(default=ConfigPhase.FROZEN, init=False)
    
    def __post_init__(self):
        """Cross-config validation and optimization."""
        # Apply cross-config rules
        if self.agent.ultra_fast and not self.optimization.headless:
            # Need to create new visualization config with disabled=True
            object.__setattr__(self, 'visualization', 
                replace(self.visualization, enabled=False))
        
        if self.optimization.headless:
            object.__setattr__(self, 'visualization',
                replace(self.visualization, enabled=False))
        
        # Validate cross-config constraints
        if self.agent.is_fast_mode and self.agent.lookahead_depth != 1:
            raise ValueError("Fast modes require lookahead_depth=1")
        
        if self.reward.type == 'l2_white_penalty' and self.reward.white_penalty is None:
            raise ValueError("white_penalty required for l2_white_penalty")
    
    def with_runtime_overrides(self, **overrides) -> 'WanderlineConfig':
        """Create new config with runtime overrides (for resume functionality)."""
        return replace(self, **overrides)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'motif_path': self.motif_path,
            'resume_from': self.resume_from,
            'agent': {
                'type': self.agent.type,
                'lookahead_depth': self.agent.lookahead_depth,
                'n_samples': self.agent.n_samples,
                'fast_mode': self.agent.fast_mode,
                'ultra_fast': self.agent.ultra_fast,
            },
            'visualization': {
                'enabled': self.visualization.enabled,
                'mode': self.visualization.mode,
                'update_frequency': self.visualization.update_frequency,
                'window_size': list(self.visualization.window_size),
                'save_snapshots': self.visualization.save_snapshots,
                'snapshot_interval': self.visualization.snapshot_interval,
            },
            'reward': {
                'type': self.reward.type,
                'white_penalty': self.reward.white_penalty,
            },
            'drawing': {
                'steps': self.drawing.steps,
                'duration': self.drawing.duration,
                'opacity': self.drawing.opacity,
                'line_width': self.drawing.line_width,
                'ratio': self.drawing.ratio,
            },
            'optimization': {
                'memory_efficient': self.optimization.memory_efficient,
                'patience': self.optimization.patience,
                'min_delta': self.optimization.min_delta,
                'verbose': self.optimization.verbose,
                'headless': self.optimization.headless,
            }
        }


# Configuration Lifecycle Management

class ConfigurationLifecycle:
    """Manages configuration lifecycle and transitions."""
    
    @staticmethod
    def create_from_sources(json_path: Optional[str] = None, args=None) -> WanderlineConfig:
        """Create configuration from JSON and CLI args with proper lifecycle."""
        
        # Phase 1: Construction (mutable)
        builder = ConfigBuilder()
        
        if json_path and Path(json_path).exists():
            builder.from_json(json_path)
        
        if args:
            builder.from_args(args)
        
        # Phase 2: Validation & Build (becomes immutable)
        config = builder.build()
        
        return config
    
    @staticmethod
    def create_for_resume(base_config: WanderlineConfig, resume_data: Dict[str, Any]) -> WanderlineConfig:
        """Create configuration for resume with necessary overrides."""
        # This is the ONLY safe time to override config after creation
        return base_config.with_runtime_overrides(
            motif_path=resume_data.get('motif_path'),
            drawing=replace(base_config.drawing,
                          steps=resume_data.get('steps', base_config.drawing.steps),
                          opacity=resume_data.get('opacity', base_config.drawing.opacity))
        )


# Usage Examples:

def example_usage():
    """Demonstrate proper configuration lifecycle."""
    
    # Phase 1: Construction (mutable)
    config = (ConfigBuilder()
              .motif_path("assets/sample.png")
              .agent(ultra_fast=True, lookahead_depth=1)
              .drawing(steps=500, opacity=0.3)
              .optimization(memory_efficient=True)
              .build())  # Phase 2: Becomes immutable
    
    # Phase 3: Runtime (read-only)
    print(f"Agent type: {config.agent.type}")
    print(f"Steps: {config.drawing.steps}")
    print(f"Visualization enabled: {config.visualization.enabled}")  # False due to ultra_fast
    
    # This would raise ImmutableConfigError:
    # config.drawing.steps = 1000  # ❌ Frozen dataclass
    
    # Proper way to create variations:
    fast_config = replace(config, 
                         drawing=replace(config.drawing, steps=1000))
    
    # For resume functionality:
    resume_data = {'steps': 750, 'opacity': 0.5}
    resumed_config = ConfigurationLifecycle.create_for_resume(config, resume_data)


if __name__ == "__main__":
    example_usage()