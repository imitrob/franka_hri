
# Human-Robot Interaction for Franka Emika Panda robot.

Command your robot with voice commands and hand gestures. Steps:
1. Teach new robotic actions: skills (as kinesthetic demonstrations)
2. Create user profile: Which skills to execute when certain action word or gesture is observed
3. Execute skills!
4. (Optional) Use LLM Merger to merge voice commands and hand gestures into single narrative

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
pip install librosa
pip install fuzzywuzzy

cd ..
colcon build --symlink-install #--cmake-args -DPython3_FIND_VIRTUALENV=ONLY
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
ln -s ~/lfd_ws/src/teleop_gesture_toolbox/scene_getter/scene_getter/scene_makers/scenes ~/lfd_ws/build/scene_getter/scene_getter/scene_makers/scenes
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
Play with [llm.py:ROLE_DESCRIPTION](llm_merger/models/llm.py)


# TRANSFORMERGE

Usage:
1. `ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=casper`
2. `ros2 llm_merger llm_merger --name_user casper`

Parameters:
1. Set of Gestures: See the `teleop_gesture_toolbox:README.md` on how to create new gestures
  - You get set of pose-gestures and gesture swipes (use default set)
  - Tune the gesture activation time: `gestures_lib.py:GestureDataDetection.activate_length` (should be user-calibrated)
  - Ignored gestures that won't trigger execution: `gestures_processor.py:GestureSentence.ignored_gestures` (think about what you need)
2. Set of Skills and Scene Object recognition: See the `franka_learning_from_demonstrations_ros2:README.md` on how to record new skills and save new scene object detection as a new template. (const)
3. Scene objects setup. Choore or define scene properties: `scenes/scene_1.yaml` and change `scene_getter.scene_makers.mocked_scene_maker.py:SCENE_FILE` (const scene set)
4. User preferences: `hri_manager/links/<username>_links.yaml` (var)
5. Merger params:
  - `--name_user`, The user name, `default="casper"` (var)
  - `--name_model`, The user name, `default="SultanR/SmolTulu-1.7b-Reinforced"`
  - `--dry_run`, Dont play skills, `default=True`
  - `--role_version`, Role description `default="v1"`
  - `--temperature`, temperature, 0.0 is deterministic, `default=0.0`
  - `--top_p`, top p', `default=1.0`
  - `--repetition_penalty`, `default=1.1`
  - `--max_new_tokens`, max words output, `default=50`

Notes:
- Gesture episode starts when hand is observed with sensor and ends when hand no longer observed, if any gesture activated, the gesture data "episode" are sent
- Within gesture episode, you can make action gestures or deictic gestures (point to objects).
- Pointing gesture (raised point finger) activates Deictic gesture, scene object selection, defined at: `gesture_processor.py:AdaptiveSetup.adaptive_setup`.
- In this version, scene object locations (for pointing gesture) are set as constants (`scene_getter.scene_makers.mocked_scene_maker.py` loads scene from `scenes` folder).
- When doing execution, the correct location of scene object is improved based on localizer.

## GESTURE-NLU: User study narration

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
- User adjusts the slider of gesture activation

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

- [ ] LM translator feature: Human description of roundness is converted by LM to parameter float 0-1

ukazat akci pour a pak akci pour s parameterem = 0. Par rict userovi o provedeni teto nove akce s tim parametrem



