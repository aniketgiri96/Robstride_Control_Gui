"""Ensures the project root is importable so ``pytest`` finds ``robstride_gui``."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
