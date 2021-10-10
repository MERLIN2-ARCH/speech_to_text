# STT (Speech To Text)

## Description

- ROS2 Foxy Fitzroy
- ROS2 speech to text system
- This microphone of the robot is used
- Python libs:
  - speech_recognition
  - nltk
  - contractions
  - pyjsgf


## Installation

```
cd ~/ros2_ws/src
git clone speech_to_text
cd speech_to_text
./install_dependencies.sh
cd ~/ros2_ws
colcon build && source install/setup.bash
```

## Launch

- Python launch
```shell
ros2 launch speech_to_text speech_to_text_launch.py
```

- XML launch
```shell
ros2 launch speech_to_text speech_to_text_launch.xml
```

## Shell Example
```shell
ros2 action send_goal /speech_to_text/listen_once speech_to_text_interfaces/action/ListenOnce {}
```