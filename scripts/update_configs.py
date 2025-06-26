#!/usr/bin/env python3
"""
CLI tool for updating configuration files.

Usage examples:
    # Update all configs
    python scripts/update_configs.py --set steps=1000 line_width=4
    
    # Update specific files
    python scripts/update_configs.py --set opacity=0.5 --files default.json long_run.json
    
    # Update with pattern
    python scripts/update_configs.py --set duration=10.0 --pattern "quick_*.json"
    
    # Dry run to see changes
    python scripts/update_configs.py --set steps=500 --dry-run
    
    # Show differences from backups
    python scripts/update_configs.py --show-diff
    
    # Restore from backups
    python scripts/update_configs.py --restore
"""

import argparse
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from wanderline.config_updater import update_configs, show_config_diff, restore_configs


def parse_set_args(set_args):
    """Parse key=value arguments into a dictionary."""
    changes = {}
    
    for arg in set_args:
        if '=' not in arg:
            print(f"❌ Invalid format: {arg}. Use key=value format.")
            sys.exit(1)
        
        key, value = arg.split('=', 1)
        
        # Try to convert to appropriate type
        if value.lower() == 'true':
            changes[key] = True
        elif value.lower() == 'false':
            changes[key] = False
        elif value.lower() == 'null':
            changes[key] = None
        else:
            # Try numeric conversion
            try:
                # Try int first
                if '.' not in value:
                    changes[key] = int(value)
                else:
                    changes[key] = float(value)
            except ValueError:
                # Keep as string
                changes[key] = value
    
    return changes


def main():
    parser = argparse.ArgumentParser(
        description="Update configuration files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument(
        '--set', nargs='+', metavar='KEY=VALUE',
        help='Set configuration values (e.g., --set steps=1000 opacity=0.5)'
    )
    
    parser.add_argument(
        '--files', nargs='+', metavar='FILE',
        help='Specific config files to update (e.g., --files default.json long_run.json)'
    )
    
    parser.add_argument(
        '--pattern', metavar='PATTERN',
        help='Glob pattern to match files (e.g., --pattern "quick_*.json")'
    )
    
    parser.add_argument(
        '--config-dir', default='configs',
        help='Directory containing config files (default: configs)'
    )
    
    parser.add_argument(
        '--no-backup', action='store_true',
        help='Skip creating backup files'
    )
    
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Show what would be changed without actually updating'
    )
    
    parser.add_argument(
        '--show-diff', action='store_true',
        help='Show differences between current configs and backups'
    )
    
    parser.add_argument(
        '--restore', action='store_true',
        help='Restore all config files from backups'
    )
    
    args = parser.parse_args()
    
    # Handle special commands
    if args.show_diff:
        show_config_diff(args.config_dir)
        return
    
    if args.restore:
        print("🔄 Restoring config files from backups...")
        results = restore_configs(args.config_dir)
        success_count = sum(results.values())
        total_count = len(results)
        print(f"\n✅ Restored {success_count}/{total_count} files")
        return
    
    # Validate arguments for update operation
    if not args.set:
        print("❌ No changes specified. Use --set to specify changes.")
        parser.print_help()
        sys.exit(1)
    
    # Parse changes
    changes = parse_set_args(args.set)
    
    if args.dry_run:
        print("🔍 Dry run - showing what would be changed:")
    else:
        print("🔧 Updating configuration files...")
    
    print(f"Changes: {changes}")
    
    # Update configs
    results = update_configs(
        changes=changes,
        files=args.files,
        pattern=args.pattern,
        config_dir=args.config_dir,
        backup=not args.no_backup,
        dry_run=args.dry_run
    )
    
    # Show summary
    success_count = sum(results.values())
    total_count = len(results)
    
    if args.dry_run:
        print(f"\n🔍 Would update {total_count} files")
    else:
        print(f"\n✅ Updated {success_count}/{total_count} files")
        if not args.no_backup:
            print("💾 Backup files created (.bak)")


if __name__ == "__main__":
    main()