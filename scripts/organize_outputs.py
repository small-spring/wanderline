#!/usr/bin/env python3
"""
Output Organization Script for Wanderline

Organizes scattered output files into a clean structure:
- Archives old runs by date
- Cleans up incomplete runs
- Provides summary statistics
- Offers cleanup options for disk space management
"""

import os
import shutil
import json
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple


def analyze_output_directory(outputs_dir: Path) -> Dict:
    """Analyze the current state of the outputs directory."""
    analysis = {
        'total_dirs': 0,
        'complete_runs': 0,
        'incomplete_runs': 0,
        'snapshot_files': 0,
        'total_size_mb': 0,
        'dirs_by_date': {},
        'incomplete_dirs': [],
        'large_dirs': []
    }
    
    for item in outputs_dir.iterdir():
        if item.is_dir() and item.name.startswith('2025'):
            analysis['total_dirs'] += 1
            
            # Extract date from directory name (YYYYMMDD_HHMMSS)
            date_part = item.name[:8]  # YYYYMMDD
            if date_part not in analysis['dirs_by_date']:
                analysis['dirs_by_date'][date_part] = []
            analysis['dirs_by_date'][date_part].append(item.name)
            
            # Check if run is complete (has final_canvas.png and drawing.mp4)
            has_final = (item / 'final_canvas.png').exists()
            has_video = (item / 'drawing.mp4').exists()
            
            if has_final and has_video:
                analysis['complete_runs'] += 1
            else:
                analysis['incomplete_runs'] += 1
                analysis['incomplete_dirs'].append(item.name)
            
            # Count snapshot files
            live_snapshots = item / 'live_snapshots'
            if live_snapshots.exists():
                snapshot_count = len(list(live_snapshots.glob('step_*.png')))
                analysis['snapshot_files'] += snapshot_count
                
                # Flag directories with many snapshots (>100)
                if snapshot_count > 100:
                    analysis['large_dirs'].append((item.name, snapshot_count))
            
            # Calculate directory size
            dir_size = sum(f.stat().st_size for f in item.rglob('*') if f.is_file())
            analysis['total_size_mb'] += dir_size / (1024 * 1024)
    
    return analysis


def create_organized_structure(outputs_dir: Path, dry_run: bool = True) -> None:
    """Create organized directory structure by date."""
    archive_dir = outputs_dir / 'archive'
    
    if not dry_run:
        archive_dir.mkdir(exist_ok=True)
    
    moves = []
    
    for item in outputs_dir.iterdir():
        if item.is_dir() and item.name.startswith('2025') and item.name != 'archive':
            # Extract date (YYYYMMDD)
            date_part = item.name[:8]
            date_archive = archive_dir / date_part
            
            if not dry_run:
                date_archive.mkdir(exist_ok=True)
                shutil.move(str(item), str(date_archive / item.name))
            
            moves.append(f"{item.name} → archive/{date_part}/{item.name}")
    
    if dry_run:
        print(f"DRY RUN: Would move {len(moves)} directories:")
        for move in moves[:10]:  # Show first 10
            print(f"  {move}")
        if len(moves) > 10:
            print(f"  ... and {len(moves) - 10} more")
    else:
        print(f"Moved {len(moves)} directories to organized archive structure")


def cleanup_incomplete_runs(outputs_dir: Path, incomplete_dirs: List[str], dry_run: bool = True) -> None:
    """Remove incomplete runs that only have partial files."""
    removed = []
    
    for dir_name in incomplete_dirs:
        dir_path = outputs_dir / dir_name
        if not dir_path.exists():
            continue
            
        # Only remove if it has very few files (likely interrupted early)
        files = list(dir_path.rglob('*'))
        if len(files) <= 3:  # Only motif.png and maybe 1-2 other files
            if not dry_run:
                shutil.rmtree(dir_path)
            removed.append(dir_name)
    
    if dry_run:
        print(f"DRY RUN: Would remove {len(removed)} incomplete directories")
        for name in removed[:5]:
            print(f"  {name}")
        if len(removed) > 5:
            print(f"  ... and {len(removed) - 5} more")
    else:
        print(f"Removed {len(removed)} incomplete directories")


def cleanup_excessive_snapshots(outputs_dir: Path, large_dirs: List[Tuple[str, int]], dry_run: bool = True) -> None:
    """Remove excessive snapshot files, keeping only every 10th snapshot."""
    cleaned = []
    
    for dir_name, snapshot_count in large_dirs:
        if snapshot_count < 50:  # Skip if not too many
            continue
            
        dir_path = outputs_dir / dir_name / 'live_snapshots'
        if not dir_path.exists():
            continue
        
        snapshots = sorted(dir_path.glob('step_*.png'))
        to_remove = []
        
        # Keep every 10th snapshot (step_000100, step_000200, etc.)
        for i, snapshot in enumerate(snapshots):
            step_num = int(snapshot.stem.split('_')[1])
            if step_num % 1000 != 0:  # Keep every 1000 steps
                to_remove.append(snapshot)
        
        if not dry_run:
            for snapshot in to_remove:
                snapshot.unlink()
        
        cleaned.append((dir_name, len(to_remove), snapshot_count))
    
    if dry_run:
        print(f"DRY RUN: Would clean snapshots in {len(cleaned)} directories:")
        for name, removed, total in cleaned:
            print(f"  {name}: {removed}/{total} snapshots")
    else:
        total_removed = sum(removed for _, removed, _ in cleaned)
        print(f"Cleaned {total_removed} snapshot files from {len(cleaned)} directories")


def main():
    parser = argparse.ArgumentParser(description='Organize Wanderline output files')
    parser.add_argument('--outputs-dir', default='outputs', help='Output directory path')
    parser.add_argument('--organize', action='store_true', help='Organize by date into archive/')
    parser.add_argument('--cleanup-incomplete', action='store_true', help='Remove incomplete runs')
    parser.add_argument('--cleanup-snapshots', action='store_true', help='Reduce excessive snapshots')
    parser.add_argument('--dry-run', action='store_true', help='Preview changes without applying')
    parser.add_argument('--analyze-only', action='store_true', help='Only show analysis, no changes')
    
    args = parser.parse_args()
    
    outputs_dir = Path(args.outputs_dir)
    if not outputs_dir.exists():
        print(f"Error: Output directory '{outputs_dir}' does not exist")
        return
    
    # Always analyze first
    print("🔍 Analyzing output directory...")
    analysis = analyze_output_directory(outputs_dir)
    
    print(f"\n📊 Analysis Results:")
    print(f"  Total directories: {analysis['total_dirs']}")
    print(f"  Complete runs: {analysis['complete_runs']}")
    print(f"  Incomplete runs: {analysis['incomplete_runs']}")
    print(f"  Total snapshot files: {analysis['snapshot_files']}")
    print(f"  Total size: {analysis['total_size_mb']:.1f} MB")
    
    print(f"\n📅 Runs by date:")
    for date, dirs in sorted(analysis['dirs_by_date'].items()):
        print(f"  {date}: {len(dirs)} runs")
    
    if analysis['large_dirs']:
        print(f"\n📸 Directories with many snapshots:")
        for dir_name, count in analysis['large_dirs'][:5]:
            print(f"  {dir_name}: {count} snapshots")
        if len(analysis['large_dirs']) > 5:
            print(f"  ... and {len(analysis['large_dirs']) - 5} more")
    
    if args.analyze_only:
        return
    
    # Apply requested operations
    if args.organize:
        print(f"\n🗂️  Organizing by date...")
        create_organized_structure(outputs_dir, args.dry_run)
    
    if args.cleanup_incomplete:
        print(f"\n🧹 Cleaning incomplete runs...")
        cleanup_incomplete_runs(outputs_dir, analysis['incomplete_dirs'], args.dry_run)
    
    if args.cleanup_snapshots:
        print(f"\n📸 Cleaning excessive snapshots...")
        cleanup_excessive_snapshots(outputs_dir, analysis['large_dirs'], args.dry_run)
    
    if args.dry_run:
        print(f"\n💡 To apply changes, run again without --dry-run")


if __name__ == '__main__':
    main()