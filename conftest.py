"""
pytest configuration and fixtures for Wanderline project.
Automatically resolves Python path issues for test imports.
"""

import sys
import os
from pathlib import Path

def pytest_configure(config):
    """
    Automatically configure Python path for test imports.
    This runs before any tests and ensures wanderline module can be imported.
    """
    # Find project root (directory containing this conftest.py)
    project_root = Path(__file__).parent.absolute()
    
    # Add project root to Python path if not already there
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    
    # Verify wanderline module can be imported
    try:
        import wanderline
        print(f"✅ Wanderline module imported successfully from {project_root}")
    except ImportError as e:
        print(f"❌ Failed to import wanderline module: {e}")
        print(f"   Project root: {project_root}")
        print(f"   Python path: {sys.path}")
        raise