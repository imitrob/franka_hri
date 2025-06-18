
# Human-Robot Interaction for Franka Emika Panda robot.

Command your robot with voice commands and hand gestures. Steps:
1. Teach new robotic actions: skills (as kinesthetic demonstrations)
2. Create user profile: Which skills to execute when certain action word or gesture is observed
3. Execute skills!
4. Use **TransforMerger** ([multi_modal_reasoning](#transformerger) package) to merge voice commands and hand gestures into single narrative

## Install 

```
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/imitrob/franka_hri.git
git clone https://github.com/imitrob/franka_learning_from_demonstrations_ros2
git clone https://github.com/imitrob/natural_language_processing.git
git clone https://github.com/imitrob/teleop_gesture_toolbox.git --depth 1

conda env create -f franka_hri/environment.yml
conda activate gesturenlu
# Hack to use whisperx with Python3.11
pip install pyannote.audio==3.3.2 --no-deps
pip install -r <(pip show pyannote.audio | grep Requires | cut -d ' ' -f2- | tr ', ' '\n' | grep -v torchaudio)
pip install whisperx --no-deps

cd ..
colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY 
source install/setup.bash

bash src/teleop_gesture_toolbox/gesture_detector/leap_motion_install.sh
```
ROS2 installs the packages to build folder. Make a symbolic links to use materials such as trajectories, configs, templates.
```
ln -s ~/robot_ws/src/franka_learning_from_demonstrations_ros2/object_localization/cfg ~/robot_ws/build/object_localization/cfg
ln -s ~/robot_ws/src/franka_learning_from_demonstrations_ros2/object_localization/config ~/robot_ws/build/object_localization/config
ln -s ~/robot_ws/src/franka_learning_from_demonstrations_ros2/trajectory_data/trajectories ~/robot_ws/build/trajectory_data/trajectories
ln -s ~/robot_ws/src/franka_hri/hri_manager/links ~/robot_ws/build/hri_manager/links
#rm    ~/robot_ws/build/gesture_detector/gesture_detector/saved_models
ln -s ~/robot_ws/src/teleop_gesture_toolbox/gesture_detector/saved_models ~/robot_ws/build/gesture_detector/gesture_detector/saved_models
ln -s ~/robot_ws/src/teleop_gesture_toolbox/scene_getter/scene_getter/scene_makers/scenes ~/robot_ws/build/scene_getter/scene_getter/scene_makers/scenes
```

`alias lfdenv='conda activate lfd'; source ~/<your_ws>/install/setup.bash`

(optional) Check your recorder device: Run `natural_language_processing/natural_language_processing/speech_to_text/audio_recorder.py`, and listen to your record back.

## Usage

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
Play with [llm.py:ROLE_DESCRIPTION](multi_modal_reasoning/models/llm.py)


# TransforMerger

Usage:
1. `sudo leapd` Gesture sensor backend
2. `ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=demo` Gesture detectors
2. `ros2 run ulti_modal_reasoning multi_modal_reasoning --name_user demo` 

Parameters:
1. Common-Set of Gestures set by default: See the `teleop_gesture_toolbox:README.md` on how to create new gestures
     - You get set of pose-gestures and gesture swipes (recommended: use default set)
     - Tune the gesture activation time: `gestures_lib.py:GestureDataDetection.activate_length` (recommended: should be user-calibrated)
     - Ignored gestures that won't trigger execution: `gestures_processor.py:GestureSentence.ignored_gestures` (note: be aware that these gestures are ignored)
2. Set of Skills and Scene Object recognition: See the `franka_learning_from_demonstrations_ros2:README.md` on how to record new skills and save new scene object detection as a new template. (recommended: for new setup, create your own set of skills)
3. Scene objects setup. Choore or define scene properties: `scenes/scene_1.yaml` and change `scene_getter.scene_makers.mocked_scene_maker.py:SCENE_FILE` (const scene set)
4. User preferences: `hri_manager/links/<username>_links.yaml` (var)
5. Merger params:
  - `--name_user`, The user name, `default="casper"` (var)
  - `--name_model`, The user name, `default="SultanR/SmolTulu-1.7b-Instruct"`
  - `--dry_run`, Dont play skills, `default=True`
  - `--temperature`, 0.0 is deterministic, `default=0.0`
  - `--top_p`, `default=1.0`, 
  - `--repetition_penalty`, `default=1.1`, 
  - `--max_new_tokens`, max words output, `default=1000`
  - Choose role description `version` manually at `role_setup.py`
  - `--config_name`, defines valid actions for constraining the skillcommand


Notes:
- Gesture episode starts when hand is observed with sensor and ends when hand no longer observed, if any gesture activated, the gesture data "episode" are sent
- Press `+` on your keyboard to start listening to voice commands
- Within gesture episode, you can make action gestures or deictic gestures (point to objects).
- Pointing gesture (raised point finger) activates Deictic gesture, scene object selection, defined at: `gesture_processor.py:AdaptiveSetup.adaptive_setup`.
- In this version, scene object locations (for pointing gesture) are set as constants (`scene_getter.scene_makers.mocked_scene_maker.py` loads scene from `scenes` folder).
- When doing execution, the correct location of scene object is improved based on localizer.

## (optional) Visualization 

Visualize dependencies across existing skills: `lfdenv; python franka_hri/hri_manager/monitor_dashboards/visualize_links.py` and see browser at `localhost:8077`
Visualize skills (from franka_learning_from_demonstration_ros2): `lfdenv; python franka_learning_from_demonstrations_ros2/trajectory_data/skill_visualizer.py` and see `localhost:8076`
TODO: Visualize merge logs: `lfdenv; python franka_hri/hri_manager/monitor_dashboards/visualize_merges.py` and see `localhost:8075`

(super-optional) What I like is to creating shortcut links by using script [/home/imitlearn/lfd_ws/src/franka_hri/hri_manager/install_accessible_links.sh](see here) `sudo bash franka_hri/hri_manager/install_accessible_links.sh`, then you don't have to remember the port: `http://skill_viewer`, `http://skill_deps`, `http://merge_log`.

