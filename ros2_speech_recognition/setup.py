from glob import glob
import os
from setuptools import setup

package_name = 'ros2_speech_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'grammars'), glob('grammars/*.gram')),
        (os.path.join('share', package_name), glob('launch/_launch.py')),
        (os.path.join('share', package_name), glob('launch/*.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='miguel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_node = ros2_speech_recognition.stt_node:main',
            'nlp_node = ros2_speech_recognition.nlp_node:main',
            'parser_node = ros2_speech_recognition.parser_node:main',
            'dialog_manager_node = ros2_speech_recognition.dialog_manager_node:main'
        ],
    },
)
