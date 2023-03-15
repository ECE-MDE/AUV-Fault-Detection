import os
from pathlib import Path


def get_newest_subdir(path: Path):
    all_subdirs = [d for d in os.listdir(path.stem) if os.path.isdir(d)]
    return max(all_subdirs, key=os.path.getmtime)