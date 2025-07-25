#!/usr/bin/env python3
"""
Pen Trail Analysis Script

Analyzes CSV logs to identify when and why pen position and trail diverge.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
from pathlib import Path
import sys

def load_latest_csv(directory="coordinate_analysis"):
    """Load the most recent CSV log file."""
    log_dir = Path(directory)
    if not log_dir.exists():
        print(f"❌ Directory {directory} not found")
        return None
        
    csv_files = list(log_dir.glob("pen_trail_analysis_*.csv"))
    if not csv_files:
        print(f"❌ No CSV files found in {directory}")
        return None
        
    # Get the most recent file
    latest_file = max(csv_files, key=lambda f: f.stat().st_mtime)
    print(f"📊 Loading: {latest_file}")
    
    try:
        df = pd.read_csv(latest_file)
        print(f"✅ Loaded {len(df)} data points")
        return df, latest_file
    except Exception as e:
        print(f"❌ Error loading CSV: {e}")
        return None

def calculate_distances(df):
    """Calculate various distance metrics."""
    # Distance between target and actual pixels
    df['pixel_error'] = np.sqrt(
        (df['target_pixel_x'] - df['actual_pixel_x'])**2 + 
        (df['target_pixel_y'] - df['actual_pixel_y'])**2
    )
    
    # Distance between pen body and trail contact points
    df['pen_trail_distance'] = np.sqrt(
        (df['pen_body_x'] - df['trail_contact_x'])**2 + 
        (df['pen_body_y'] - df['trail_contact_y'])**2 + 
        (df['pen_body_z'] - df['trail_contact_z'])**2
    )
    
    # XY distance only (horizontal divergence)
    df['pen_trail_xy_distance'] = np.sqrt(
        (df['pen_body_x'] - df['trail_contact_x'])**2 + 
        (df['pen_body_y'] - df['trail_contact_y'])**2
    )
    
    return df

def create_analysis_plots(df, output_dir="coordinate_analysis"):
    """Create comprehensive analysis plots."""
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Pen Trail Analysis - Position Divergence Over Time', fontsize=16)
    
    # Plot 1: Pixel error over time
    axes[0, 0].plot(df['step_count'], df['pixel_error'], 'b-', alpha=0.7)
    axes[0, 0].set_title('Target vs Actual Pixel Position Error')
    axes[0, 0].set_xlabel('Step Count')
    axes[0, 0].set_ylabel('Pixel Error (px)')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Plot 2: Pen-Trail distance over progress
    axes[0, 1].plot(df['progress_percent'], df['pen_trail_xy_distance'], 'r-', alpha=0.7)
    axes[0, 1].set_title('Pen Body vs Trail Contact Distance (XY)')
    axes[0, 1].set_xlabel('Circle Progress (%)')
    axes[0, 1].set_ylabel('XY Distance (m)')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Joint angles over time (first 3 joints)
    axes[1, 0].plot(df['step_count'], df['joint_0'], label='Shoulder Pan', alpha=0.7)
    axes[1, 0].plot(df['step_count'], df['joint_1'], label='Shoulder Lift', alpha=0.7)
    axes[1, 0].plot(df['step_count'], df['joint_2'], label='Elbow', alpha=0.7)
    axes[1, 0].set_title('Robot Joint Angles Over Time')
    axes[1, 0].set_xlabel('Step Count')
    axes[1, 0].set_ylabel('Joint Angle (rad)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Plot 4: 2D trajectory comparison
    axes[1, 1].plot(df['pen_body_x'], df['pen_body_y'], 'b-', label='Pen Body', alpha=0.7)
    axes[1, 1].plot(df['trail_contact_x'], df['trail_contact_y'], 'r-', label='Trail Contact', alpha=0.7)
    axes[1, 1].set_title('2D Trajectory Comparison')
    axes[1, 1].set_xlabel('X Position (m)')
    axes[1, 1].set_ylabel('Y Position (m)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_aspect('equal')
    
    plt.tight_layout()
    
    # Save plot
    output_path = Path(output_dir) / "pen_trail_analysis.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"📊 Analysis plot saved: {output_path}")
    
    plt.show()

def find_divergence_point(df, threshold=0.01):
    """Find when pen and trail start to diverge significantly."""
    # Smooth the distance to reduce noise
    window_size = min(20, len(df) // 10)
    if window_size < 3:
        window_size = 3
        
    df['smooth_distance'] = df['pen_trail_xy_distance'].rolling(
        window=window_size, center=True
    ).mean()
    
    # Find first point where distance exceeds threshold
    divergence_points = df[df['smooth_distance'] > threshold]
    
    if len(divergence_points) > 0:
        first_divergence = divergence_points.iloc[0]
        print(f"\n🔍 DIVERGENCE DETECTED:")
        print(f"   Step: {first_divergence['step_count']}")
        print(f"   Progress: {first_divergence['progress_percent']:.1f}%")
        print(f"   XY Distance: {first_divergence['pen_trail_xy_distance']:.4f}m")
        print(f"   Pixel Error: {first_divergence['pixel_error']:.2f}px")
        
        return first_divergence
    else:
        print(f"\n✅ No significant divergence found (threshold: {threshold}m)")
        return None

def print_summary_stats(df):
    """Print summary statistics."""
    print(f"\n📊 SUMMARY STATISTICS:")
    print(f"   Total steps: {len(df)}")
    print(f"   Max pixel error: {df['pixel_error'].max():.2f}px")
    print(f"   Mean pixel error: {df['pixel_error'].mean():.2f}px")
    print(f"   Max pen-trail XY distance: {df['pen_trail_xy_distance'].max():.4f}m")
    print(f"   Mean pen-trail XY distance: {df['pen_trail_xy_distance'].mean():.4f}m")
    print(f"   Circle completion: {df['progress_percent'].max():.1f}%")

def main():
    parser = argparse.ArgumentParser(description='Analyze pen trail CSV logs')
    parser.add_argument('--csv', help='Specific CSV file to analyze')
    parser.add_argument('--threshold', type=float, default=0.01, 
                       help='Divergence threshold in meters (default: 0.01)')
    parser.add_argument('--no-plot', action='store_true', 
                       help='Skip creating plots')
    
    args = parser.parse_args()
    
    # Load data
    if args.csv:
        try:
            df = pd.read_csv(args.csv)
            csv_file = Path(args.csv)
            print(f"📊 Loading specified file: {csv_file}")
        except Exception as e:
            print(f"❌ Error loading {args.csv}: {e}")
            return
    else:
        result = load_latest_csv()
        if result is None:
            return
        df, csv_file = result
    
    # Calculate metrics
    df = calculate_distances(df)
    
    # Analysis
    print_summary_stats(df)
    divergence_point = find_divergence_point(df, args.threshold)
    
    # Create plots
    if not args.no_plot:
        create_analysis_plots(df)
    
    print(f"\n✅ Analysis complete! Check the plots for visual insights.")

if __name__ == '__main__':
    main()