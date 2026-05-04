#!/usr/bin/env python3
"""
Simplified server startup script to avoid import issues
"""
import os
import sys

# Add all necessary paths
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(project_root, 'src', 'backend', 'core'))
sys.path.insert(0, os.path.join(project_root, 'src', 'backend', 'optimization'))
sys.path.insert(0, os.path.join(project_root, 'config'))

# Now import and run the main server
if __name__ == "__main__":
    import uvicorn
    
    print("Starting ACO-ITS Server...")
    uvicorn.run("src.backend.api.main:app", host="0.0.0.0", port=8000, reload=True)
