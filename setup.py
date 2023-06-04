import os
from glob import glob

from setuptools import setup

package_name = "ultralytics_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alpaca-zip",
    maintainer_email="zip@todo.todo",
    description="The ultralytics_ros package",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["tracker_node = ultralytics_ros.tracker_node:main"],
    },
)
