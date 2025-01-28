
# Franka HRI

Parts:
1. Make link from gesture and nl input to robotic action.
2. Execute commands

## Install 

```
mkdir -p lfd_ws/src
cd lfd_ws/src
git clone git@gitlab.ciirc.cvut.cz:imitrob/franka_hri.git
git clone https://github.com/imitrob/franka_learning_from_demonstrations_ros2
git clone https://github.com/ichores-research/natural_language_processing.git
git clone https://github.com/imitrob/teleop_gesture_toolbox.git --branch mediapipe --depth 1

conda env create -f franka_hri/environment.yml
conda activate lfd

cd ..
colcon build --symlink-install
source install/setup.bash

bash teleop_gesture_toolbox/gesture_detector/leap_motion_install.sh
```
ROS2 installs the packages to build folder. Make a symbolic links to use materials such as trajectories, configs, templates.
```
ln -s ~/lfd_ws/src/franka_learning_from_demonstrations_ros2/object_localization/cfg ~/lfd_ws/build/object_localization/cfg
ln -s ~/lfd_ws/src/franka_learning_from_demonstrations_ros2/object_localization/config ~/lfd_ws/build/object_localization/config
ln -s ~/lfd_ws/src/franka_learning_from_demonstrations_ros2/trajectory_data/trajectories ~/lfd_ws/build/trajectory_data/trajectories
ln -s ~/lfd_ws/src/franka_hri/hri_manager/links ~/lfd_ws/build/hri_manager/links
ln -s ~/lfd_ws/src/teleop_gesture_toolbox/gesture_detector/saved_models ~/lfd_ws/build/gesture_detector/gesture_detector/saved_models
```

`alias lfdenv='conda activate lfd'; source ~/<your_ws>/install/setup.bash`


## Part 1: Link gestures to actions

```
lfdenv; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=melichar
```

```
lfdenv; ros2 launch object_localization box_localization_launch.py
```

```
lfdenv; ros2 run hri_manager link_gesture_to_action --name_user Melichar --name_skill touch_sponge
```
- Assign name of the user and name of the skills
- Check the link `links` folder for specific user if it is correct

## Part 2: HRI Magnum Opus

```
lfdenv; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user:=Melichar
```

```
lfdenv; ros2 launch object_localization box_localization_launch.py
```

```
lfdenv; ros2 run hri_manager action_executor
```




## TODOs:

- [ ] Note somewhere -> I changed 1. default mapping and 2. ignored gestures 
- [ ] tutorial at the start of experiment with the instruction video (show the reach of gestures)
- [ ] questionare
- [ ] rotated circle for proceeding gestures
- [ ] beginning of experiment - calibration
- [ ] adjust slider of gesture activation
- [ ] one experiment - users pick their own gestures versus user not to pick gestures


