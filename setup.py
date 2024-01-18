import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = "ultralytics_ros"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alpaca-zip",
    maintainer_email="alpaca.zip@gmail.com",
    description="The ultralytics_ros package",
    license="AGPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["tracker_node = ultralytics_ros.tracker_node:main"],
    },
)
