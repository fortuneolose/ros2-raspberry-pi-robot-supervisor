"""Run ament's Jazzy flake8 configuration over maintained package sources."""

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8() -> None:
    """Require ROS2-010 supervisor Python sources to pass ament_flake8."""
    config = Path(__file__).parents[1] / 'ament_flake8.ini'
    paths = ['robot_supervisor', 'launch', 'setup.py', 'test']
    return_code, errors = main_with_errors(
        argv=['--config', str(config), *paths]
    )
    assert return_code == 0, \
        'Found %d code style errors:\n' % len(errors) + '\n'.join(errors)
