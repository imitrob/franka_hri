
# Franka HRI

Parts:
1. Make link from gesture and nl input to robotic action.
2. Execute commands

## Install 

```
mkdir -p lfd_ros2_ws/src
cd lfd_ros2_ws/src
git clone https://github.com/imitrob/franka_learning_from_demonstration_ros2

bash teleop_gesture_toolbox/gesture_detector/leap_motion_install.sh


conda env create -f franka_learning_from_demonstrations/environment.yml
conda activate gesturenlu
cd ..
colcon build --symlink-install
source install/setup.bash
```
ROS2 installs the packages to build folder. Make a symbolic links to use materials such as trajectories, configs, templates.
```
ln -s ~/lfd_ros2_ws/src/franka_learning_from_demonstrations/object_localization/cfg ~/lfd_ros2_ws/build/object_localization/cfg
ln -s ~/lfd_ros2_ws/src/franka_learning_from_demonstrations/object_localization/config ~/lfd_ros2_ws/build/object_localization/config
ln -s ~/lfd_ros2_ws/src/franka_learning_from_demonstrations/trajectory_data/trajectories ~/lfd_ros2_ws/build/trajectory_data/trajectories
ln -s ~/lfd_ros2_ws/src/hri_manager/links ~/lfd_ros2_ws/build/hri_manager/links
```

`alias hrienv='conda activate gesturenlu'; source ~/<your_ws>/install/setup.bash`


## Part 1: Link gestures to actions

```
hrienv; leap; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=Melichar
```

```
hrienv; ros2 launch object_localization box_localization_launch.py
```

```
hrienv; ros2 run hri_manager link_gesture_to_action --name_user Melichar --name_skill touch_sponge
```
- Assign name of the user and name of the skills
- Check the link `links` folder for specific user if it is correct

## Part 2: HRI Magnum Opus

```
hrienv; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user:=Melichar
```

```
hrienv; ros2 launch object_localization box_localization_launch.py
```

```
hrienv; ros2 run hri_manager action_executor
```




TODO 
- [ ] Note somewhere -> I changed 1. default mapping and 2. ignored gestures 
- [ ] When the all things are running the processor is full, cannot even recognize static gestures properly

## My laptop microphone was not working, I fixed by copying the system libraries 

`mkdir ~/miniconda3/envs/<conda env>/lib/alsa-lib/`
`sudo cp /usr/lib/x86_64-linux-gnu/alsa-lib/* ~/miniconda3/envs/<conda env>/lib/alsa-lib/`


one experiment - users pick their own gestures versus user not to pick gestures

early trial - test system on someone


beginning of experiment - calibration
- adjust slider of gesture activation


rotated circle for proceeding gestures


tutorial at the start of experiment with the instruction video
- show the cone


questionare

