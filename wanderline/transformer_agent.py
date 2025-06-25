"""
Transformer-based agent for multi-step planning in drawing tasks.
This is the initial implementation focusing on the basic architecture.
まだ使わない
"""

import torch
import torch.nn as nn
import numpy as np
from typing import Tuple, List
from wanderline.canvas import apply_stroke
from wanderline.reward import compute_reward


class StateEncoder(nn.Module):
    """
    Encodes the current state (canvas, motif, position, history) into feature vectors.
    """
    
    def __init__(self, canvas_shape: Tuple[int, int], feature_dim: int = 128):
        super().__init__()
        self.canvas_shape = canvas_shape
        self.feature_dim = feature_dim
        
        # Simple CNN for image features (will be enhanced later)
        self.cnn = nn.Sequential(
            nn.Conv2d(3, 32, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.AdaptiveAvgPool2d((8, 8)),
            nn.Flatten(),
            nn.Linear(64 * 8 * 8, feature_dim)
        )
        
        # Position encoding
        self.pos_encoder = nn.Linear(2, feature_dim // 4)
        
        # Action history encoding (simple for now)
        self.history_encoder = nn.Linear(10, feature_dim // 4)  # last 10 actions
        
        # Feature fusion
        total_input_dim = feature_dim * 2 + feature_dim // 4 + feature_dim // 4
        self.fusion = nn.Linear(total_input_dim, feature_dim)
        
    def forward(self, canvas: torch.Tensor, motif: torch.Tensor, 
                position: torch.Tensor, history: torch.Tensor) -> torch.Tensor:
        """
        Encode the current state into a feature vector.
        
        Args:
            canvas: (B, 3, H, W) current canvas
            motif: (B, 3, H, W) target motif
            position: (B, 2) current position
            history: (B, 10) recent action history
            
        Returns:
            features: (B, feature_dim) encoded state
        """
        # Extract visual features
        canvas_feat = self.cnn(canvas)
        motif_feat = self.cnn(motif)
        
        # Encode position and history
        pos_feat = self.pos_encoder(position)
        hist_feat = self.history_encoder(history)
        
        # Combine all features
        combined = torch.cat([canvas_feat, motif_feat, pos_feat, hist_feat], dim=-1)
        return self.fusion(combined)


class TransformerAgent:
    """
    Transformer-based agent for multi-step planning.
    Initial version focuses on basic functionality and integration.
    """
    
    def __init__(
            self, canvas_shape: Tuple[int, int], sequence_length: int = 10, 
            feature_dim: int = 128, device: str = 'cpu'
            ):
        self.canvas_shape = canvas_shape
        self.sequence_length = sequence_length
        self.feature_dim = feature_dim
        self.device = device
        
        # Initialize encoder
        self.state_encoder = StateEncoder(canvas_shape, feature_dim).to(device)
        
        # Simple MLP for now (will be replaced with actual transformer)
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1)  # Output single angle
        ).to(device)
        
        # Action history buffer
        self.action_history = []
        
    def choose_next_angle(
            self, prev_canvas: np.ndarray, motif: np.ndarray, 
            start_pt: Tuple[int, int], length: int) -> float:
        """
        Choose the next angle using the transformer model.
        For now, this is a simplified version that will be enhanced.
        """
        # Convert to tensors
        canvas_tensor = self._numpy_to_tensor(prev_canvas)
        motif_tensor = self._numpy_to_tensor(motif)
        position_tensor = torch.tensor(
            [start_pt[0] / self.canvas_shape[1], 
            start_pt[1] / self.canvas_shape[0]], 
            dtype=torch.float32, device=self.device).unsqueeze(0)
        
        # Pad action history to fixed length
        history_padded = self._pad_history()
        history_tensor = torch.tensor(history_padded, dtype=torch.float32, 
                                    device=self.device
                                    ).unsqueeze(0)
        
        # Encode state
        with torch.no_grad():
            state_features = self.state_encoder(
                canvas_tensor, motif_tensor, 
                position_tensor, history_tensor
                )
            angle_raw = self.policy_net(state_features)
            angle = (torch.sigmoid(angle_raw) * 2 * np.pi).cpu().item()
        
        # Update action history
        self.action_history.append(angle)
        if len(self.action_history) > 10:
            self.action_history.pop(0)
            
        return angle
    
    def _numpy_to_tensor(self, img: np.ndarray) -> torch.Tensor:
        """Convert numpy image to tensor."""
        if img.ndim == 3:
            # Convert BGR to RGB and normalize
            img_rgb = img[:, :, ::-1] / 255.0
            tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float()
        else:
            tensor = torch.from_numpy(img / 255.0).unsqueeze(0).float()
        return tensor.unsqueeze(0).to(self.device)
    
    def _pad_history(self) -> List[float]:
        """Pad action history to fixed length."""
        padded = self.action_history.copy()
        while len(padded) < 10:
            padded.insert(0, 0.0)
        return padded[-10:]  # Keep only last 10


class GreedyAgent:
    """
    Refactored greedy agent for baseline comparison.
    """
    
    def __init__(self):
        pass
    
    def choose_next_angle(
            self, prev_canvas: np.ndarray, motif: np.ndarray, 
            start_pt: Tuple[int, int], length: int, 
            n_samples: int = 36) -> float:
        """
        Choose the angle that maximizes immediate reward.
        This is the existing greedy implementation.
        """
        best_angle = 0.0
        best_reward = float('-inf')
        
        for angle in np.linspace(0, 2 * np.pi, n_samples, endpoint=False):
            try:
                next_canvas, _ = apply_stroke(prev_canvas, angle, start=start_pt, 
                                            length=length, return_end=True)
                reward = compute_reward(prev_canvas, next_canvas, motif)
            except Exception:
                continue
                
            if reward > best_reward:
                best_reward = reward
                best_angle = float(angle)
                
        return best_angle
