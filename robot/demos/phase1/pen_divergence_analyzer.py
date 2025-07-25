#!/usr/bin/env python3
"""
Pen Divergence Analyzer

Analyzes pen body vs trail contact position divergence from CSV logs.
No external dependencies - works in Docker environments.

Purpose: Identify when and why the blue pen position and red trail diverge
during circle drawing, especially the reported issue at 50% completion.
"""

import csv
import math
from pathlib import Path
import argparse

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
        data = []
        with open(latest_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Convert numeric fields
                for key in row:
                    if key != 'timestamp':
                        try:
                            row[key] = float(row[key])
                        except ValueError:
                            pass
                data.append(row)
        
        print(f"✅ Loaded {len(data)} data points")
        return data, latest_file
    except Exception as e:
        print(f"❌ Error loading CSV: {e}")
        return None

def calculate_distances(data):
    """Calculate various distance metrics."""
    for row in data:
        # Distance between target and actual pixels
        pixel_dx = row['target_pixel_x'] - row['actual_pixel_x']
        pixel_dy = row['target_pixel_y'] - row['actual_pixel_y']
        row['pixel_error'] = math.sqrt(pixel_dx**2 + pixel_dy**2)
        
        # Distance between actual pen tip and trail display points
        pen_dx = row['pen_tip_x'] - row['trail_display_x']
        pen_dy = row['pen_tip_y'] - row['trail_display_y']
        pen_dz = row['pen_tip_z'] - row['trail_display_z']
        row['pen_trail_distance'] = math.sqrt(pen_dx**2 + pen_dy**2 + pen_dz**2)
        
        # XY distance only (horizontal divergence)
        row['pen_trail_xy_distance'] = math.sqrt(pen_dx**2 + pen_dy**2)
    
    return data

def find_divergence_point(data, threshold=0.01):
    """Find when pen and trail start to diverge significantly."""
    # Simple moving average for smoothing
    window_size = min(10, len(data) // 20)
    if window_size < 3:
        window_size = 3
    
    for i in range(window_size, len(data)):
        # Calculate moving average
        avg_distance = sum(
            data[j]['pen_trail_xy_distance'] 
            for j in range(i - window_size, i)
        ) / window_size
        
        if avg_distance > threshold:
            divergence_row = data[i]
            print(f"\n🔍 DIVERGENCE DETECTED:")
            print(f"   Step: {divergence_row['step_count']:.0f}")
            print(f"   Progress: {divergence_row['progress_percent']:.1f}%")
            print(f"   XY Distance: {divergence_row['pen_trail_xy_distance']:.4f}m")
            print(f"   Pixel Error: {divergence_row['pixel_error']:.2f}px")
            print(f"   Target Pixel: ({divergence_row['target_pixel_x']:.1f}, {divergence_row['target_pixel_y']:.1f})")
            print(f"   Actual Pixel: ({divergence_row['actual_pixel_x']:.1f}, {divergence_row['actual_pixel_y']:.1f})")
            
            return divergence_row
    
    print(f"\n✅ No significant divergence found (threshold: {threshold}m)")
    return None

def print_summary_stats(data):
    """Print summary statistics."""
    if not data:
        return
        
    pixel_errors = [row['pixel_error'] for row in data]
    xy_distances = [row['pen_trail_xy_distance'] for row in data]
    progress_values = [row['progress_percent'] for row in data]
    
    print(f"\n📊 SUMMARY STATISTICS:")
    print(f"   Total steps: {len(data)}")
    print(f"   Max pixel error: {max(pixel_errors):.2f}px")
    print(f"   Mean pixel error: {sum(pixel_errors)/len(pixel_errors):.2f}px")
    print(f"   Max pen-trail XY distance: {max(xy_distances):.4f}m")
    print(f"   Mean pen-trail XY distance: {sum(xy_distances)/len(xy_distances):.4f}m")
    print(f"   Circle completion: {max(progress_values):.1f}%")

def analyze_by_progress(data):
    """Analyze divergence at different progress levels."""
    print(f"\n📈 PROGRESS-BASED ANALYSIS:")
    
    progress_ranges = [
        (0, 25, "First Quarter (0-25%)"),
        (25, 50, "Second Quarter (25-50%)"),
        (50, 75, "Third Quarter (50-75%)"),
        (75, 100, "Fourth Quarter (75-100%)")
    ]
    
    for min_prog, max_prog, label in progress_ranges:
        range_data = [
            row for row in data 
            if min_prog <= row['progress_percent'] < max_prog
        ]
        
        if range_data:
            avg_pixel_error = sum(row['pixel_error'] for row in range_data) / len(range_data)
            avg_xy_distance = sum(row['pen_trail_xy_distance'] for row in range_data) / len(range_data)
            max_xy_distance = max(row['pen_trail_xy_distance'] for row in range_data)
            
            print(f"   {label}:")
            print(f"     Average pixel error: {avg_pixel_error:.2f}px")
            print(f"     Average XY distance: {avg_xy_distance:.4f}m")
            print(f"     Max XY distance: {max_xy_distance:.4f}m")

def find_worst_divergence_points(data, top_n=5):
    """Find the worst divergence points."""
    sorted_data = sorted(data, key=lambda x: x['pen_trail_xy_distance'], reverse=True)
    
    print(f"\n🔍 TOP {top_n} WORST DIVERGENCE POINTS:")
    for i, row in enumerate(sorted_data[:top_n]):
        print(f"   #{i+1}: Step {row['step_count']:.0f} ({row['progress_percent']:.1f}%)")
        print(f"        XY Distance: {row['pen_trail_xy_distance']:.4f}m")
        print(f"        Pixel Error: {row['pixel_error']:.2f}px")
        print(f"        Joint 0 (Pan): {row['joint_0']:.3f} rad")
        print(f"        Joint 1 (Lift): {row['joint_1']:.3f} rad")

def export_simple_report(data, output_file="analysis_report.txt"):
    """Export a simple text report."""
    try:
        with open(output_file, 'w') as f:
            f.write("PEN TRAIL ANALYSIS REPORT\n")
            f.write("=" * 40 + "\n\n")
            
            f.write(f"Total data points: {len(data)}\n")
            
            if data:
                pixel_errors = [row['pixel_error'] for row in data]
                xy_distances = [row['pen_trail_xy_distance'] for row in data]
                
                f.write(f"Max pixel error: {max(pixel_errors):.2f}px\n") 
                f.write(f"Mean pixel error: {sum(pixel_errors)/len(pixel_errors):.2f}px\n")
                f.write(f"Max pen-trail distance: {max(xy_distances):.4f}m\n")
                f.write(f"Mean pen-trail distance: {sum(xy_distances)/len(xy_distances):.4f}m\n")
                
                f.write("\nWorst 10 divergence points:\n")
                sorted_data = sorted(data, key=lambda x: x['pen_trail_xy_distance'], reverse=True)
                for i, row in enumerate(sorted_data[:10]):
                    f.write(f"{i+1}. Step {row['step_count']:.0f}: {row['pen_trail_xy_distance']:.4f}m "
                           f"({row['progress_percent']:.1f}%)\n")
        
        print(f"📄 Report exported: {output_file}")
    except Exception as e:
        print(f"⚠️ Failed to export report: {e}")

def main():
    parser = argparse.ArgumentParser(description='Analyze pen body vs trail contact divergence')
    parser.add_argument('--csv', help='Specific CSV file to analyze')
    parser.add_argument('--threshold', type=float, default=0.01, 
                       help='Divergence threshold in meters (default: 0.01)')
    parser.add_argument('--report', help='Export text report to file')
    
    args = parser.parse_args()
    
    # Load data
    if args.csv:
        try:
            data = []
            with open(args.csv, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    for key in row:
                        if key != 'timestamp':
                            try:
                                row[key] = float(row[key])
                            except ValueError:
                                pass
                    data.append(row)
            csv_file = Path(args.csv)
            print(f"📊 Loading specified file: {csv_file}")
        except Exception as e:
            print(f"❌ Error loading {args.csv}: {e}")
            return
    else:
        result = load_latest_csv()
        if result is None:
            return
        data, csv_file = result
    
    # Calculate metrics
    data = calculate_distances(data)
    
    # Analysis
    print_summary_stats(data)
    divergence_point = find_divergence_point(data, args.threshold)
    analyze_by_progress(data)
    find_worst_divergence_points(data)
    
    # Export report if requested
    if args.report:
        export_simple_report(data, args.report)
    
    print(f"\n✅ Analysis complete!")

if __name__ == '__main__':
    main()