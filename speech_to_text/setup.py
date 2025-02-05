from glob import glob
import os
from setuptools import setup

package_name = "speech_to_text"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "grammars"), glob("grammars/*.gram")),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miguel",
    maintainer_email="miguel@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stt_node = speech_to_text.stt_node:main",
            "nlp_node = speech_to_text.nlp_node:main",
            "parser_node = speech_to_text.parser_node:main",
            "dialog_manager_node = speech_to_text.dialog_manager_node:main",
        ],
    },
)
