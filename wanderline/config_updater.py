import json
import os
import glob
from typing import Dict, List, Union, Optional
from pathlib import Path


def update_configs(
    changes: Dict[str, Union[str, int, float, bool, None]],
    files: Optional[List[str]] = None,
    pattern: Optional[str] = None,
    config_dir: str = "configs",
    backup: bool = True,
    dry_run: bool = False,
    validate_keys: bool = False
) -> Dict[str, bool]:
    """
    Update configuration files with new values.
    
    Args:
        changes: Dictionary of key-value pairs to update
        files: Specific config files to update (relative to config_dir)
        pattern: Glob pattern to match files (e.g., "quick_*.json")
        config_dir: Directory containing config files
        backup: Create .bak backup files before updating
        dry_run: Show what would be changed without actually updating
        validate_keys: Validate that keys exist in at least one config file
        
    Returns:
        Dictionary mapping file paths to success status
        
    Examples:
        # Update specific files
        update_configs({"steps": 1000}, files=["default.json", "long_run.json"])
        
        # Update all quick test configs
        update_configs({"opacity": 0.5}, pattern="quick_*.json")
        
        # Update all configs
        update_configs({"line_width": 4}, pattern="*.json")
    """
    
    # Validate keys if requested
    if validate_keys:
        _validate_config_keys(changes, config_dir)
    
    # Determine which files to update
    config_path = Path(config_dir)
    target_files = []
    
    if files:
        target_files = [config_path / f for f in files]
    elif pattern:
        target_files = list(config_path.glob(pattern))
    else:
        target_files = list(config_path.glob("*.json"))
    
    results = {}
    
    for file_path in target_files:
        try:
            # Read current config
            if not file_path.exists():
                results[file_path.name] = False
                print(f"❌ File not found: {file_path}")
                continue
                
            with open(file_path, 'r') as f:
                config = json.load(f)
            
            # Apply changes
            original_config = config.copy()
            config.update(changes)
            
            if dry_run:
                print(f"🔍 {file_path.name}:")
                for key, new_value in changes.items():
                    old_value = original_config.get(key, "<not set>")
                    if old_value != new_value:
                        print(f"  {key}: {old_value} → {new_value}")
                results[file_path.name] = True
                continue
            
            # Create backup if requested
            if backup:
                backup_path = file_path.with_suffix('.json.bak')
                with open(backup_path, 'w') as f:
                    json.dump(original_config, f, indent=2)
            
            # Write updated config
            with open(file_path, 'w') as f:
                json.dump(config, f, indent=2)
            
            results[file_path.name] = True
            print(f"✅ Updated: {file_path.name}")
            
        except Exception as e:
            results[file_path.name] = False
            print(f"❌ Error updating {file_path.name}: {e}")
    
    return results


def show_config_diff(config_dir: str = "configs"):
    """Show differences between current configs and their backups."""
    config_path = Path(config_dir)
    
    for backup_file in config_path.glob("*.json.bak"):
        original_file = backup_file.with_suffix('')
        
        if not original_file.exists():
            continue
            
        try:
            with open(backup_file, 'r') as f:
                old_config = json.load(f)
            with open(original_file, 'r') as f:
                new_config = json.load(f)
            
            print(f"\n📄 {original_file.name}:")
            
            # Find differences
            all_keys = set(old_config.keys()) | set(new_config.keys())
            has_changes = False
            
            for key in sorted(all_keys):
                old_val = old_config.get(key, "<not set>")
                new_val = new_config.get(key, "<not set>")
                
                if old_val != new_val:
                    print(f"  {key}: {old_val} → {new_val}")
                    has_changes = True
            
            if not has_changes:
                print("  No changes")
                
        except Exception as e:
            print(f"❌ Error comparing {original_file.name}: {e}")


def restore_configs(config_dir: str = "configs") -> Dict[str, bool]:
    """Restore all config files from their backups."""
    config_path = Path(config_dir)
    results = {}
    
    for backup_file in config_path.glob("*.json.bak"):
        original_file = backup_file.with_suffix('')
        
        try:
            # Copy backup to original
            with open(backup_file, 'r') as f:
                config = json.load(f)
            with open(original_file, 'w') as f:
                json.dump(config, f, indent=2)
            
            results[str(original_file)] = True
            print(f"🔄 Restored: {original_file.name}")
            
        except Exception as e:
            results[str(original_file)] = False
            print(f"❌ Error restoring {original_file.name}: {e}")
    
    return results


def _validate_config_keys(changes: Dict[str, Union[str, int, float, bool, None]], config_dir: str):
    """Validate that config keys exist in at least one config file."""
    config_path = Path(config_dir)
    valid_keys = set()
    
    # Collect all keys from existing config files
    for config_file in config_path.glob("*.json"):
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
                valid_keys.update(config.keys())
        except Exception:
            continue  # Skip invalid config files
    
    # Check if any change keys are invalid
    invalid_keys = set(changes.keys()) - valid_keys
    if invalid_keys:
        raise ValueError(f"Invalid config key(s): {', '.join(invalid_keys)}")