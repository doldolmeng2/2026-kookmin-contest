"""Backward-compatible entry point for the safe manual-drive launch."""

import importlib.util
from pathlib import Path


def generate_launch_description():
    launch_path = Path(__file__).with_name('manual_drive.py')
    spec = importlib.util.spec_from_file_location(
        'manual_drive_safe_launch',
        launch_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()
