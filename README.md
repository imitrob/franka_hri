
# Human-Robot Interaction for Franka Emika Panda robot.

Command your robot with voice commands and hand gestures. Steps:
1. Teach new robotic actions: skills (as kinesthetic demonstrations)
2. Create user profile: Which skills to execute when certain action word or gesture is observed
3. Execute skills!

LLM Merger:
1. 
## Install 

```
mkdir -p lfd_ws/src
cd lfd_ws/src
git clone git@gitlab.ciirc.cvut.cz:imitrob/franka_hri.git
git clone https://github.com/imitrob/franka_learning_from_demonstrations_ros2
git clone https://github.com/ichores-research/natural_language_processing.git
git clone https://github.com/imitrob/teleop_gesture_toolbox.git --depth 1

conda env create -f franka_hri/environment.yml
conda activate lfd
# Hack to use whisperx with Python3.11
pip install pyannote.audio==3.3.2 --no-deps
pip install -r <(pip show pyannote.audio | grep Requires | cut -d ' ' -f2- | tr ', ' '\n' | grep -v torchaudio)
pip install whisperx --no-deps

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
rm ~/lfd_ws/build/gesture_detector/gesture_detector/saved_models
ln -s ~/lfd_ws/src/teleop_gesture_toolbox/gesture_detector/saved_models ~/lfd_ws/build/gesture_detector/gesture_detector/saved_models
```

`alias lfdenv='conda activate lfd'; source ~/<your_ws>/install/setup.bash`


## Usage

1. Check the recorder driver is correct at `audio_recorder.py`

## Part 1: Skills:

Record your own set of skills: 
0. Move to home `ros2 launch skills_manager home_launch.py`
1. Record template `ros2 launch object_localization record_template_launch.py template_name:=<your template>`
2. Record skill 1. `ros2 launch object_localization box_localization_launch.py` 2. `ros2 launch skills_manager record_skill_launch.py name_skill:=<your skill>`
Note: if record skill and you want to record it again, always do homing before the new recording attempt.
1. Play the skill 1. `ros2 launch object_localization box_localization_launch.py` 2. `ros2 launch skills_manager plau_skill_launch.py name_skill:=<your skill> name_template:=<your template>`
See saved skills in `trajectory_data/trajectories` folder.

## Part 2: Link gestures to actions

1. `lfdenv; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=casper`
2. `lfdenv; ros2 launch object_localization box_localization_launch.py`
3. `lfdenv; ros2 run hri_manager link_gesture_to_action --name_user casper --name_skill touch_sponge`

- Note: Run `localhost:8000` to see which gestures are available
- Assign name of the user and name of the skills
- Check the new link in `hri_manager/links` folder if it is correct
- Or you may want to adjust the links `.yaml` based on your needs

## (optional) Part 3: Check the commands for exeucting

1. `lfdenv; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=casper`
2. `lfdenv; ros2 launch object_localization box_localization_launch.py`
3. Test: `lfdemv; ros2 run hri_manager action_executor_dry_run --name_user casper` Run: `lfdemv; ros2 run hri_manager action_executor --name_user casper` 

Tuning:
Play with [sentence_processor.py:ROLE_DESCRIPTION](natural_language_processing/natural_language_processing/sentence_instruct_transformer/sentence_processor.py)





## LLM Merger

Usage:
1. `ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=casper`
2. `ros2 llm_merger llm_merger --name_user casper`

## User study narration

### Introduction


- Introduce the reach of the gestures sensor
- Single gesture has a pose and a swipe
- You can choose of poses: 1. Fist, 2. Pinch, 3. Point, 4. Two, 5. Three, 6. Four, 7. Five, 8. Thumb, (9. Holding)
- You can choose swipe direction: 1. Left, 2. Right, 3. Up, 4. Down, (5. Forward, 6. Backward, 7. Pour)
  - Say that no-moving dynamic gesture is not allowed
- Absolute position of hand is not used, e.g., hand orientation up or down


### Calibration

- Try some gestures
- Tune the activation parameter
- [ ] User adjusts the slider of gesture activation


### Linking the commands

- Make sure that after the user links the gesture, the orientation to the gesture must not change!
- Make sure that the users not show some moving in the gesture

### Action Gestures

- [ ] idea - extension - users pick their own gestures versus user not to pick gestures

### Teach parametric gestures

- Show the length of the gestures influences the skill movement

### Action Gestures + Parameter

- The robot shows the first skill (parameter = 0), then the robot shows the alternative skill (parameter = 1)
- User shows the gesture and sees the results - feedback
- User shows it again, see the improvement

### User study finish

- [ ] questionare



## TODOs:

- [ ] Note somewhere -> I changed 1. default mapping and 2. ignored gestures 
- [ ] LM translator feature: Human description of roundness is converted by LM to parameter float 0-1

ukazat akci pour a pak akci pour s parameterem = 0. Par rict userovi o provedeni teto nove akce s tim parametrem
