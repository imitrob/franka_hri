import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """Initialise rclpy exactly once for the whole pytest session (shared by
    all test files -- per-file init fixtures collide when several files run
    in one session)."""
    rclpy.init()
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass
