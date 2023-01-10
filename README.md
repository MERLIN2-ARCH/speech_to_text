# STT (Speech To Text)

STT for ROS 2 using JSGF grammars.

## Installation

```shell
# clone
$ cd ~/ros2_ws/src
$ git clone https://github.com/MERLIN2-ARCH/speech_to_text.git

# dependencies
$ sudo apt-get install -y python-dev libportaudio2 libportaudiocpp0 portaudio19-dev libasound-dev swig
$ cd speech_to_text
$ pip3 install -r requirements.txt
$ python3 ./nltk_download.py

# colcon
$ cd ~/ros2_ws
$ colcon build
```

## Usage

### Launch

```shell
$ ros2 launch speech_to_text speech_to_text.launch.py
```

### Shell Example

```shell
$ ros2 action send_goal /speech_to_text/listen_once speech_to_text_interfaces/action/ListenOnce {}
```
