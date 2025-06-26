"""
Immutable Configuration System for Wanderline
=============================================

Provides type-safe, immutable configuration with clear lifecycle management.
Configuration becomes immutable after calling freeze() on the builder.
"""

from dataclasses import dataclass, field, replace
from typing import Optional, Literal, Dict, Any
from pathlib import Path
import json
from enum import Enum


class ConfigPhase(Enum):
    """Configuration lifecycle phases."""
    CONSTRUCTION = "construction"  # Building config from sources
    VALIDATION = "validation"     # Cross-validation and constraints  
    FROZEN = "frozen"            # Immutable, ready for use


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
            raise ValueError(f"update_frequency must be > 0, got {self.update_frequency}")
        if self.snapshot_interval <= 0:
            raise ValueError(f"snapshot_interval must be > 0, got {self.snapshot_interval}")


@dataclass(frozen=True)
class RewardConfig:
    """Immutable reward configuration."""
    type: Literal['l2', 'l2_white_penalty', 'l2_coverage_bonus', 'l2_exploration_bonus'] = 'l2'
    white_penalty: Optional[float] = None
    
    def __post_init__(self):
        if self.type == 'l2_white_penalty' and self.white_penalty is None:
            raise ValueError("white_penalty required when using l2_white_penalty")
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
            raise ValueError(f"steps must be non-negative, got {self.steps}")
        if not (0.0 < self.ratio <= 1.0):
            raise ValueError(f"ratio must be > 0 and <= 1, got {self.ratio}")
        if not (0.0 <= self.opacity <= 1.0):
            raise ValueError(f"opacity must be 0-1, got {self.opacity}")


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
            raise ValueError(f"patience must be non-negative, got {self.patience}")
        if self.min_delta < 0:
            raise ValueError(f"min_delta must be non-negative, got {self.min_delta}")


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
        
        # Main config
        if hasattr(args, 'motif_path') and args.motif_path:
            self._motif_path = args.motif_path
        if hasattr(args, 'resume_from') and args.resume_from:
            self._resume_from = args.resume_from
            
        # Agent config
        if hasattr(args, 'agent_type'):
            self._agent_data['type'] = args.agent_type
        if hasattr(args, 'lookahead_depth'):
            self._agent_data['lookahead_depth'] = args.lookahead_depth
        if hasattr(args, 'n_samples') and args.n_samples:
            self._agent_data['n_samples'] = args.n_samples
        if hasattr(args, 'fast_agent'):
            self._agent_data['fast_mode'] = args.fast_agent
        if hasattr(args, 'ultra_fast'):
            self._agent_data['ultra_fast'] = args.ultra_fast
            
        # Visualization config
        if hasattr(args, 'visualization') and isinstance(args.visualization, dict):
            self._viz_data.update(args.visualization)
        if hasattr(args, 'headless'):
            # Map headless to visualization.enabled
            if args.headless:
                self._viz_data['enabled'] = False
                
        # Reward config
        if hasattr(args, 'reward_type'):
            self._reward_data['type'] = args.reward_type
        if hasattr(args, 'white_penalty') and args.white_penalty is not None:
            self._reward_data['white_penalty'] = args.white_penalty
            
        # Drawing config
        if hasattr(args, 'steps'):
            self._drawing_data['steps'] = args.steps
        if hasattr(args, 'duration'):
            self._drawing_data['duration'] = args.duration
        if hasattr(args, 'opacity'):
            self._drawing_data['opacity'] = args.opacity
        if hasattr(args, 'line_width'):
            self._drawing_data['line_width'] = args.line_width
        if hasattr(args, 'ratio'):
            self._drawing_data['ratio'] = args.ratio
            
        # Optimization config
        if hasattr(args, 'memory_efficient'):
            self._opt_data['memory_efficient'] = args.memory_efficient
        if hasattr(args, 'patience'):
            self._opt_data['patience'] = args.patience
        if hasattr(args, 'min_delta'):
            self._opt_data['min_delta'] = args.min_delta
        if hasattr(args, 'verbose'):
            self._opt_data['verbose'] = args.verbose
        if hasattr(args, 'headless'):
            self._opt_data['headless'] = args.headless
            
        return self
    
    def freeze(self) -> 'WanderlineConfig':
        """Freeze configuration and create immutable instance."""
        if self._phase != ConfigPhase.CONSTRUCTION:
            raise ImmutableConfigError(f"Config already frozen in {self._phase.value} phase")
        
        self._phase = ConfigPhase.VALIDATION
        
        # Create individual configs with validation
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
    """Immutable main configuration for Wanderline system."""
    motif_path: Optional[str] = None
    resume_from: Optional[str] = None
    agent: AgentConfig = field(default_factory=AgentConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    reward: RewardConfig = field(default_factory=RewardConfig)
    drawing: DrawingConfig = field(default_factory=DrawingConfig)
    optimization: OptimizationConfig = field(default_factory=OptimizationConfig)
    
    def __post_init__(self):
        """Cross-config validation and optimization."""
        # Apply cross-config rules for ultra-fast and headless modes
        viz_disabled = False
        
        if self.agent.ultra_fast and not self.optimization.headless:
            viz_disabled = True
            print("Ultra-fast mode: Real-time visualization automatically disabled for maximum performance")
        
        if self.optimization.headless:
            viz_disabled = True
            
        if viz_disabled and self.visualization.enabled:
            # Need to create new visualization config with enabled=False
            object.__setattr__(self, 'visualization', 
                replace(self.visualization, enabled=False))
        
        # Validate cross-config constraints
        if self.agent.is_fast_mode and self.agent.lookahead_depth != 1:
            raise ValueError("Fast modes only work with lookahead_depth=1")
    
    def for_resume(self, resume_data: Dict[str, Any]) -> 'WanderlineConfig':
        """Create new config for resume with necessary overrides."""
        # This is the ONLY safe way to "modify" config after freezing
        overrides = {}
        
        if 'motif_path' in resume_data:
            overrides['motif_path'] = resume_data['motif_path']
            
        # Handle drawing config overrides
        drawing_overrides = {}
        for key in ['steps', 'opacity', 'ratio', 'duration', 'line_width']:
            if key in resume_data:
                drawing_overrides[key] = resume_data[key]
        
        if drawing_overrides:
            overrides['drawing'] = replace(self.drawing, **drawing_overrides)
            
        # Handle other config overrides similarly...
        if 'greedy' in resume_data:
            # Map old 'greedy' field to agent config
            overrides['agent'] = replace(self.agent, type='greedy' if resume_data['greedy'] else 'transformer')
        
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


# Configuration Lifecycle Management Functions

def create_config_from_sources(json_path: Optional[str] = None, args=None) -> WanderlineConfig:
    """Create configuration from JSON and CLI args with proper lifecycle."""
    
    # Phase 1: Construction (mutable)
    builder = ConfigBuilder()
    
    if json_path and Path(json_path).exists():
        builder.from_json(json_path)
    
    if args:
        builder.from_args(args)
    
    # Phase 2: Validation & Freeze (becomes immutable)
    config = builder.freeze()
    
    return config


def create_config_for_resume(base_config: WanderlineConfig, resume_data: Dict[str, Any]) -> WanderlineConfig:
    """Create configuration for resume with necessary overrides."""
    return base_config.for_resume(resume_data)


# Backward Compatibility Bridge

def config_to_args_namespace(config: WanderlineConfig):
    """Convert immutable config back to args-like namespace for legacy code."""
    from argparse import Namespace
    
    return Namespace(
        motif_path=config.motif_path,
        resume_from=config.resume_from,
        
        # Agent
        agent_type=config.agent.type,
        lookahead_depth=config.agent.lookahead_depth,
        n_samples=config.agent.n_samples,
        fast_agent=config.agent.fast_mode,
        ultra_fast=config.agent.ultra_fast,
        greedy=config.agent.type == 'greedy',  # Legacy compatibility
        
        # Visualization
        visualization=config.visualization.__dict__,
        headless=config.optimization.headless,
        
        # Reward
        reward_type=config.reward.type,
        white_penalty=config.reward.white_penalty,
        
        # Drawing
        steps=config.drawing.steps,
        duration=config.drawing.duration,
        opacity=config.drawing.opacity,
        line_width=config.drawing.line_width,
        ratio=config.drawing.ratio,
        
        # Optimization
        memory_efficient=config.optimization.memory_efficient,
        patience=config.optimization.patience,
        min_delta=config.optimization.min_delta,
        verbose=config.optimization.verbose,
    )