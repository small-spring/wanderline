#!/usr/bin/env python3
"""
Hardcoded Parameter Analysis Tool

Scans the Wanderline codebase for hardcoded values and suggests which ones
should be moved to constants.py or made configurable.
"""

import os
import re
import sys
from pathlib import Path
from typing import List, Dict, Tuple

def analyze_hardcoded_values(root_dir: str) -> Dict[str, List[Tuple[int, str, str]]]:
    """
    Analyze Python files for hardcoded values.
    
    Returns:
        Dict mapping file paths to list of (line_number, line_content, issue_type)
    """
    hardcoded_patterns = {
        'magic_numbers': [
            (r'\b0\.4\b', 'Stroke length ratio (should use DEFAULT_STROKE_LENGTH_RATIO)'),
            (r'\b255\b', 'White color value (should use WHITE_COLOR_VALUE)'),
            (r'\b256\b', 'Canvas dimension (should use DEFAULT_CANVAS_SIZE)'),
            (r'\b60\b', 'Max FPS limit (should use MAX_VIDEO_FPS)'),
            (r'\b36\b', 'Default n_samples (should use DEFAULT_N_SAMPLES)'),
            (r'\b24\b', 'Performance threshold (should use LOOKAHEAD_PERFORMANCE_WARNING_THRESHOLD)'),
            (r'\b16\b', 'Progressive refinement threshold (should use PROGRESSIVE_REFINEMENT_THRESHOLD)'),
            (r'\b128\b', 'Feature dimensions (should use TRANSFORMER_FEATURE_DIM)'),
            (r'\b1000\b', 'Memory efficient threshold (should use MEMORY_EFFICIENT_RECOMMENDED_STEPS)'),
            (r'np\.pi\s*/\s*[0-9]', 'Mathematical constants (should use PI constants)'),
        ],
        'color_values': [
            (r'\(\s*0\s*,\s*0\s*,\s*0\s*\)', 'Black color (should use STROKE_COLOR)'),
            (r'==\s*255', 'White color comparison (should use WHITE_COLOR_VALUE)'),
        ],
        'array_dimensions': [
            (r'np\.ones\s*\(\s*\(\s*256\s*,\s*256', 'Default canvas size (should use DEFAULT_CANVAS_SIZE)'),
            (r'last\s+10', 'Action history length (should use ACTION_HISTORY_LENGTH)'),
        ],
        'performance_thresholds': [
            (r'n_samples\s*>\s*24', 'Performance warning threshold'),
            (r'patience\s*=\s*10', 'Early stopping patience'),
            (r'min_delta\s*=\s*1e-4', 'Minimum delta threshold'),
        ]
    }
    
    results = {}
    wanderline_dir = os.path.join(root_dir, 'wanderline')
    
    for py_file in Path(wanderline_dir).rglob('*.py'):
        if py_file.name == 'constants.py':
            continue  # Skip our constants file
            
        issues = []
        try:
            with open(py_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                
            for line_num, line in enumerate(lines, 1):
                line_stripped = line.strip()
                if line_stripped.startswith('#'):
                    continue  # Skip comments
                    
                for category, patterns in hardcoded_patterns.items():
                    for pattern, description in patterns:
                        if re.search(pattern, line):
                            issues.append((line_num, line.strip(), f"{category}: {description}"))
                            
        except Exception as e:
            print(f"Error reading {py_file}: {e}")
            
        if issues:
            results[str(py_file.relative_to(root_dir))] = issues
    
    return results

def print_analysis_report(results: Dict[str, List[Tuple[int, str, str]]]):
    """Print a formatted analysis report."""
    print("🔍 HARDCODED PARAMETER ANALYSIS REPORT")
    print("=" * 50)
    print()
    
    total_issues = sum(len(issues) for issues in results.values())
    print(f"Found {total_issues} potential hardcoded values across {len(results)} files")
    print()
    
    # Group by issue type
    issue_categories = {}
    for file_path, issues in results.items():
        for line_num, line, issue_type in issues:
            category = issue_type.split(':')[0]
            if category not in issue_categories:
                issue_categories[category] = []
            issue_categories[category].append((file_path, line_num, line, issue_type))
    
    # Print summary by category
    print("📊 SUMMARY BY CATEGORY:")
    for category, items in issue_categories.items():
        print(f"  {category}: {len(items)} occurrences")
    print()
    
    # Print detailed findings
    print("📋 DETAILED FINDINGS:")
    print()
    
    for file_path, issues in results.items():
        print(f"📁 {file_path}")
        for line_num, line, issue_type in issues:
            print(f"  Line {line_num:3d}: {issue_type}")
            print(f"           {line}")
        print()

def suggest_fixes():
    """Suggest specific fixes for common hardcoded values."""
    print("🔧 SUGGESTED FIXES:")
    print("=" * 30)
    print()
    
    fixes = [
        ("canvas.py:22", "length = int(min(w, h) * 0.4)", 
         "from .constants import DEFAULT_STROKE_LENGTH_RATIO\nlength = int(min(w, h) * DEFAULT_STROKE_LENGTH_RATIO)"),
        
        ("config_manager.py", "args.n_samples = 16/24/36", 
         "from .constants import PERFORMANCE_SAMPLES\nargs.n_samples = PERFORMANCE_SAMPLES[args.performance_mode]"),
        
        ("memory_efficient_canvas.py:193", "coarse_angles = np.array([0, np.pi/4, ...])", 
         "from .constants import PI, PI_OVER_4\ncoarse_angles = np.array([0, PI_OVER_4, PI_OVER_2, ...])"),
        
        ("drawing_engine.py:95", "max_fps = 60", 
         "from .constants import MAX_VIDEO_FPS\nmax_fps = MAX_VIDEO_FPS"),
        
        ("agent.py:64", "n_samples > 24", 
         "from .constants import LOOKAHEAD_PERFORMANCE_WARNING_THRESHOLD\nif lookahead_depth > 1 and n_samples > LOOKAHEAD_PERFORMANCE_WARNING_THRESHOLD:"),
    ]
    
    for i, (location, current, suggested) in enumerate(fixes, 1):
        print(f"{i}. {location}")
        print(f"   Current:   {current}")
        print(f"   Suggested: {suggested}")
        print()

def main():
    if len(sys.argv) != 2:
        print("Usage: python find_hardcoded.py <wanderline_root_directory>")
        sys.exit(1)
    
    root_dir = sys.argv[1]
    if not os.path.exists(root_dir):
        print(f"Error: Directory {root_dir} does not exist")
        sys.exit(1)
    
    results = analyze_hardcoded_values(root_dir)
    print_analysis_report(results)
    suggest_fixes()
    
    print("✅ Analysis complete!")
    print(f"Next steps:")
    print(f"1. Review the constants.py file created in wanderline/constants.py")
    print(f"2. Replace hardcoded values with imports from constants")
    print(f"3. Test that all functionality still works correctly")
    print(f"4. Consider making frequently-changed values configurable")

if __name__ == "__main__":
    main()