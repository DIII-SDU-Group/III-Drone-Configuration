from pathlib import Path

import pytest

import rclpy


TEST_SCHEMA_FILE = Path(__file__).resolve().parent / "resources" / "test_parameter_manifest.yaml"


@pytest.fixture(scope="session", autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()
