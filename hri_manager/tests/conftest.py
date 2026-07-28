import threading

import pytest
import rclpy


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """Initialise rclpy exactly once for the whole pytest session (per-module
    init fixtures collide when several files run in one session).

    Teardown shuts rclpy down and waits for background spin threads
    (SpinningRosNode) to notice: a spin thread still alive while the rclpy C
    layer finalizes aborts the interpreter (SIGABRT, exit code 134), which
    test runners report as a failure after all tests passed."""
    rclpy.init()
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass
    for t in threading.enumerate():
        if t is not threading.main_thread():
            t.join(timeout=2.0)
