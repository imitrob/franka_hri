# Human-Robot Interaction for Franka Emika Panda robot.

Command robot with voice + hand gestures. Steps:
1. Teach new robot actions: skills (kinesthetic demonstrations)
2. Create user profile: which skill run on which action word or gesture
3. Execute skills!
4. Use **TransforMerger** ([multi_modal_reasoning](#transformerger) package): merge voice commands + hand gestures into single narrative

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


ROS2 install packages to build folder. `*_PATH` env variables point packages to materials in src: trajectories, configs, templates, links (each package's `__init__.py` reads its variable; without it the build folder is used). Set them in the sourcing alias:
```Shell
alias lfdenv='conda activate gesturenlu;
export HRI_MANAGER_PATH=~/frankahri_ws/src/franka_hri/hri_manager;
export OBJECT_LOCALIZATION_PATH=~/frankahri_ws/src/franka_learning_from_demonstrations_ros2/object_localization;
export TRAJECTORY_DATA_PATH=~/frankahri_ws/src/franka_learning_from_demonstrations_ros2/trajectory_data;
export GESTURE_MODELS_PATH=~/frankahri_ws/src/teleop_gesture_toolbox/gesture_detector/saved_models;
export SCENES_PATH=~/frankahri_ws/src/teleop_gesture_toolbox/scene_getter/scene_getter/scene_makers/scenes;
source ~/frankahri_ws/install/setup.bash'
```

(optional) Check recorder device: run `natural_language_processing/natural_language_processing/speech_to_text/audio_recorder.py`, listen to recording.

## Usage

## Part 1: Skills (robot+camera):

Record own skill set: 
0. Move to home `ros2 launch skills_manager home_launch.py`
1. Record template `ros2 launch object_localization record_template_launch.py name_template:=<your template>`
2. Record skill 1. `ros2 launch object_localization box_localization_launch.py` 2. `ros2 launch skills_manager record_skill_launch.py name_skill:=<your skill>`
Note: re-recording skill? Always home before new recording attempt.
   1. Play skill 1. `ros2 launch object_localization box_localization_launch.py` 2. `ros2 launch skills_manager plau_skill_launch.py name_skill:=<your skill> name_template:=<your template>`
Saved skills in `trajectory_data/trajectories` folder.

## Part 2: Link gestures to actions (opt. gest+camera)

1. `lfdenv; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=casper`
2. `lfdenv; ros2 launch object_localization box_localization_launch.py`
3. `lfdenv; ros2 run hri_manager link_gesture_to_action --name_user casper --name_skill touch_sponge`

- Note: open `localhost:6357` to see available gestures (live display started by gesture detectors)
- Assign user name + skill names
- Check new link in `hri_manager/links` folder correct
- Adjust links `.yaml` as needed

## (optional) Part 3: Plain text command execution `action_executor`

Test textual input to do actions.

1. `lfdenv; ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=casper` (optional, gesture input)
2. `lfdenv; ros2 launch object_localization box_localization_launch.py`
3. Run: `lfdenv; ros2 run hri_manager action_executor --name_user casper`
   - Type command from user links vocabulary, e.g. `sponge_wipe sponge_wipe` (action + object). Exact words only — anything else rejected as invalid. By design.


# TransforMerger

Every model runs in its own server; interactive nodes only hold clients (see `hri_manager/interaction.py`). Resulting skill command published on `/hri/skill_command`; robot (LfD) node subscribes there.

Usage:
1. `vllm serve Qwen/Qwen2.5-3B-Instruct --port 8000` Start reasoning LLM server (suggested models below). Merger auto-discover served model; point elsewhere with `VLLM_BASE_URL`.
2. `ros2 run natural_language_processing stt_node` Speech-to-text server (Whisper; required for voice)
3. `ros2 run natural_language_processing tts_node` Text-to-speech server (Kokoro; optional — skipped with warning when absent)
4. `sudo leapd` Gesture sensor backend
5. `ros2 launch gesture_sentence_maker sentence_maker_launch.py sensor:=leap user_name:=demo` Gesture detectors
6. `ros2 run multi_modal_reasoning multi_modal_reasoning --name_user demo`

### Reasoning LLM (vLLM server)

LLM run in separate [vLLM](https://docs.vllm.ai) server (OpenAI-compatible API); merger just client (see `multi_modal_reasoning/models/llm.py`). Start: `vllm serve <model> --port 8000`.

Use small **instruct** (non-reasoning) models — merger force schema-constrained JSON output; plain instruct avoid empty-output failure mode of thinking models. Suggested (≈fp16 VRAM):
- `Qwen/Qwen2.5-3B-Instruct` (~6 GB) — best all-round: strong JSON adherence, no thinking mode.
- `Qwen/Qwen3-4B-Instruct-2507` (~8 GB) — instruct-only Qwen3; best on noisy/ambiguous commands.
- `meta-llama/Llama-3.2-3B-Instruct` (~6 GB) — reliable baseline.
- `microsoft/Phi-4-mini-instruct` (3.8B, ~8 GB) — strong extraction per parameter.
- `google/gemma-3-4b-it` (~8 GB) — strong instruction-following.
- `ibm-granite/granite-3.3-2b-instruct` (~4 GB) — lightweight.

Smaller (`Qwen/Qwen2.5-1.5B-Instruct`, `HuggingFaceTB/SmolLM2-1.7B-Instruct`, ~3 GB) fastest but weaker on adversarial multi-verb commands.

Parameters:
1. Common-Set of Gestures set by default: see `teleop_gesture_toolbox:README.md` for creating new gestures
     - Get pose-gestures + gesture swipes (recommended: default set)
     - Tune gesture activation time: `gestures_lib.py:GestureDataDetection.activate_length` (recommended: user-calibrated)
     - Ignored gestures, won't trigger execution: `gestures_processor.py:GestureSentence.ignored_gestures` (note: these gestures ignored)
2. Set of Skills and Scene Object recognition: see `franka_learning_from_demonstrations_ros2:README.md` for recording new skills + saving new scene object detection as template. (recommended: new setup, create own skill set)
3. Scene objects setup. Choose or define scene properties: `scenes/scene_1.yaml`, change `scene_getter.scene_makers.mocked_scene_maker.py:SCENE_FILE` (const scene set)
4. User preferences: `hri_manager/links/<username>_links.yaml` (var). Note: `actions` list not stored in yaml — derived from arity lists (`zero/single/double_object_actions` + `directional_actions`), see `hri_manager/user_links.py`
5. Merger CLI args (`ros2 run multi_modal_reasoning multi_modal_reasoning ...`):
  - `--name_user`, valid actions/objects/scene constraining SkillCommand come from `hri_manager/links/<user>_links.yaml`, `default="demo"`


Notes:
- Gesture episode start when sensor sees hand, end when hand gone; any gesture activated, gesture data "episode" sent
- Voice recording via terminal: press enter to start, enter again to stop; then enter to execute, `r` to retry
- Gesture names mapped to action words before LLM (e.g. grab -> pick): `gesture_meaning.one_to_one_mapping.OneToOneMapping` (direct call, `/teleop_gesture_toolbox/get_meaning` service too slow)
- Within gesture episode: action gestures or deictic gestures (point to objects).
- Pointing gesture (raised point finger) activate Deictic gesture, scene object selection, defined at: `gesture_processor.py:AdaptiveSetup.adaptive_setup`.
- This version: scene object locations (for pointing gesture) constant (`scene_getter.scene_makers.mocked_scene_maker.py` load scene from `scenes` folder).
- On execution, scene object location improved by localizer.

## (optional) Visualization 

Visualize dependencies across skills: `lfdenv; python src/franka_hri/hri_manager/monitor_dashboards/visualize_links.py`, browser `localhost:8077`
Visualize merge logs: `lfdenv; python src/franka_hri/multi_modal_reasoning/multi_modal_reasoning/saved_logs/visualize_merges.py`, see `localhost:8075`


# FAQ

## Q: pytests in VSCode doesn't work

- A: `launch_testing` (ROS pytest plugin, auto-loaded) incompatible with pytest 8 (new import_path(..., *, consider_namespace_packages=...) API made argument mandatory). Need pytest < 8.0.0, e.g. `conda install -c conda-forge pytest=7.4.*`. Condition already in `environment.yml`!


## Q: After installation the numpy is 2.2.6 and pip version

- A: Not wanted. Reinstall:

```
pip uninstall numpy # Confirm (Y)
mamba install -c conda-forge numpy==1.26.4 --force-reinstall # reinstall the conda numpy package
```

## Q: Kokoro installation bug:

- A: change line:
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