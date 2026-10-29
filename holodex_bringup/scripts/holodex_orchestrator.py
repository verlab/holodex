#!/usr/bin/env python3
import sys
import os

# Permite execução direta via script sem necessidade de colcon build prévio
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from holodex_bringup.orchestrator import main

if __name__ == '__main__':
    main()
