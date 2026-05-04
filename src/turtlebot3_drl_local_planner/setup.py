from glob import glob
from setuptools import find_packages, setup

package_name = "turtlebot3_drl_local_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hug",
    maintainer_email="hug@example.com",
    description="GAP_SAC-inspired DRL local planner research package.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "train_gap_sac = turtlebot3_drl_local_planner.train_gap_sac:main",
            "eval_policy = turtlebot3_drl_local_planner.eval_policy:main",
            "eval_shadow_compare = turtlebot3_drl_local_planner.eval_shadow_compare:main",
            "export_policy = turtlebot3_drl_local_planner.export_policy:main",
            "drl_controller_node = turtlebot3_drl_local_planner.nodes.drl_controller_node:main",
        ],
    },
)
