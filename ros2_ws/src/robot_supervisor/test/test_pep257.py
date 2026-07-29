"""Run ament's Jazzy PEP 257 convention over maintained package sources."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257() -> None:
    """Require ROS2-010 supervisor docstrings to pass ament_pep257."""
    paths = ['robot_supervisor', 'launch', 'setup.py', 'test']
    assert main(argv=paths) == 0
