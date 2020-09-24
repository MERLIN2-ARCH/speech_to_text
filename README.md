# ros2_speech_recognition

## Description

- ROS2 Foxy Fitzroy
- ROS2 speech recognition system.
- This system uses the microphone of the robot.
- Python libs:
  - speech_recognition
  - nltk
  - contractions
  - pyjsgf


## Installation

```
cd ~/ros2_foxy/src
git clone ros2_speech_recognition
cd ros2_speech_recognition
./install_dependencies.sh
cd ~/ros2_foxy
colcon build && source install/setup.bash
```

## Launh

- Python launch
```
ros2 launch ros2_speech_recognition ros2_speech_recognition_launch.py
```

- XML launch
```
ros2 launch ros2_speech_recognition ros2_speech_recognition_launch.xml
```
