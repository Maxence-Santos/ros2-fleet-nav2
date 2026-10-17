from glob import glob
import os

from setuptools import find_packages, setup

package_name = "fleet_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maxence",
    maintainer_email="maxence@todo.todo",
    description="Three namespaced AMRs sharing one warehouse map.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fleet_manager = fleet_nav.fleet_manager:main",
            "fleet_navigator = fleet_nav.navigator:main",
            "click_goal = fleet_nav.click_goal:main",
            "dynamic_actors = fleet_nav.dynamic_actors:main",
        ],
    },
)
