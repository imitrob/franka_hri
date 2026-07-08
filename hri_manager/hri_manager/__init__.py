
import os
path = os.path.dirname(os.path.abspath(__file__))
# HRI_MANAGER_PATH points to the package source dir (holds links/); without
# it the ROS2 build dir is used (see franka_hri README for the export)
package_path = os.path.expanduser(os.environ.get("HRI_MANAGER_PATH", "/".join(path.split("/")[:-1])))