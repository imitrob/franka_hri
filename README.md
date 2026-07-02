
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
1. Record template `ros2 launch object_localization record_template_launch.py name_template:=<your template>`
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
1. `vllm serve Qwen/Qwen2.5-3B-Instruct --port 8000` Start the reasoning LLM server (see suggested models below). The merger auto-discovers whichever model is served; point it elsewhere with `VLLM_BASE_URL`.
2. `sudo leapd` Gesture sensor backend
3. `ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=demo` Gesture detectors
4. `ros2 run multi_modal_reasoning multi_modal_reasoning`

### Reasoning LLM (vLLM server)

The LLM runs in a separate [vLLM](https://docs.vllm.ai) server (OpenAI-compatible API); the merger is just a client (see `multi_modal_reasoning/models/llm.py`). Start it with `vllm serve <model> --port 8000`.

Use small **instruct** (non-reasoning) models — the merger forces schema-constrained JSON output, so a plain instruct model avoids the empty-output failure mode of thinking models. Suggested (≈fp16 VRAM):
- `Qwen/Qwen2.5-3B-Instruct` (~6 GB) — best all-round pick: strong JSON adherence, no thinking mode.
- `Qwen/Qwen3-4B-Instruct-2507` (~8 GB) — instruct-only Qwen3; best on noisy/ambiguous commands.
- `meta-llama/Llama-3.2-3B-Instruct` (~6 GB) — reliable baseline.
- `microsoft/Phi-4-mini-instruct` (3.8B, ~8 GB) — strong extraction per parameter.
- `google/gemma-3-4b-it` (~8 GB) — strong instruction-following.
- `ibm-granite/granite-3.3-2b-instruct` (~4 GB) — lightweight.

Smaller (`Qwen/Qwen2.5-1.5B-Instruct`, `HuggingFaceTB/SmolLM2-1.7B-Instruct`, ~3 GB) run fastest but are weaker on adversarial multi-verb commands.

Parameters:
1. Common-Set of Gestures set by default: See the `teleop_gesture_toolbox:README.md` on how to create new gestures
     - You get set of pose-gestures and gesture swipes (recommended: use default set)
     - Tune the gesture activation time: `gestures_lib.py:GestureDataDetection.activate_length` (recommended: should be user-calibrated)
     - Ignored gestures that won't trigger execution: `gestures_processor.py:GestureSentence.ignored_gestures` (note: be aware that these gestures are ignored)
2. Set of Skills and Scene Object recognition: See the `franka_learning_from_demonstrations_ros2:README.md` on how to record new skills and save new scene object detection as a new template. (recommended: for new setup, create your own set of skills)
3. Scene objects setup. Choore or define scene properties: `scenes/scene_1.yaml` and change `scene_getter.scene_makers.mocked_scene_maker.py:SCENE_FILE` (const scene set)
4. User preferences: `hri_manager/links/<username>_links.yaml` (var)
5. Merger CLI args (`ros2 run multi_modal_reasoning multi_modal_reasoning ...`):
  - `--config_name`, valid actions/objects/scene that constrain the SkillCommand, `CONFIG_DEMO` or `CONFIG3`, `default="CONFIG_DEMO"`
  - `--role_version`, system-prompt template, `default="structured"` (use `structured_directions` for move commands)
  - The reasoning model is whatever the vLLM server serves (not a CLI arg). Sampling is fixed deterministic (temperature 0, greedy) in `models/llm.py`.


Notes:
- Gesture episode starts when hand is observed with sensor and ends when hand no longer observed, if any gesture activated, the gesture data "episode" are sent
- Press `+` on your keyboard to start listening to voice commands
- Within gesture episode, you can make action gestures or deictic gestures (point to objects).
- Pointing gesture (raised point finger) activates Deictic gesture, scene object selection, defined at: `gesture_processor.py:AdaptiveSetup.adaptive_setup`.
- In this version, scene object locations (for pointing gesture) are set as constants (`scene_getter.scene_makers.mocked_scene_maker.py` loads scene from `scenes` folder).
- When doing execution, the correct location of scene object is improved based on localizer.

## (optional) Visualization 

Visualize dependencies across existing skills: `lfdenv; python src/franka_hri/hri_manager/monitor_dashboards/visualize_links.py` and see browser at `localhost:8077`
Visualize skills (from franka_learning_from_demonstration_ros2): `lfdenv; python src/franka_learning_from_demonstrations_ros2/trajectory_data/trajectory_data/skill_visualizer.py` and see `localhost:8076`

Visualize merge logs: `lfdenv; python src/franka_hri/multi_modal_reasoning/multi_modal_reasoning/saved_logs/visualize_merges.py` and see `localhost:8075`

(super-optional) What I like is to creating shortcut links by using script [hri_manager/install_accessible_links.sh](see here) `sudo bash franka_hri/hri_manager/install_accessible_links.sh`, then you don't have to remember the port: `http://skills`, `http://skill_links`, `http://hri_log`.


# FAQ

## Q: pytests in VSCode doesn't work

- A: `launch_testing` (ROS pytest plugin) that’s auto-loaded isn’t compatible with pytest (the new import_path(..., *, consider_namespace_packages=...) API in pytest 8 made that argument mandatory). You need ptest version < 8.0.0, e.g., version (`conda install -c conda-forge pytest=7.4.*`), I added this condition in the `environment.yml` already!


## Q: After installation the numpy is 2.2.6 and pip version

- A: We don't want this, reinstallation:

```
pip uninstall numpy # Confirm (Y)
mamba install -c conda-forge numpy==1.26.4 --force-reinstall # reinstall the conda numpy package
```

## Q: Kokoro installation bug:

- A: change the following line:
```
/home/doma/miniconda3/envs/gesturenlu2/lib/python3.11/site-packages/misaki/espeak.py:10
```
#EspeakWrapper.set_data_path(espeakng_loader.get_data_path())
EspeakWrapper.data_path = espeakng_loader.get_data_path()
```

## Q: My Panda robot didn't move.
- A: Test homing first: `ros2 launch skills_manager home_launch.py`. I had a situation where, the script got stuck when entering the Desk. On a second run, it was fine.


## Q: The transcription is always saying `thank you`.
A: Your microphone doesn't work. See `hri_manager/tests/test_audio.py` - perhaps change the target soundcard `plughw` in `natural_language_processing/speech_to_text/audio_recorder.py` cmd variable.


