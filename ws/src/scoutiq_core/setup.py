from setuptools import find_packages, setup

package_name = "scoutiq_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="scoutiq",
    maintainer_email="dev@scoutiq.org",
    description="Core utilities, trajectory processing, and navigation nodes for ScoutIQ",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "save_data = scoutiq_core.save_data:main",
            "listener = scoutiq_core.listener:main",
            "navigate = scoutiq_core.navigate:main",
            "publish_traj = scoutiq_core.publish_traj:main",
            "extract_robot_trajectory = scoutiq_core.extract_robot_trajectory:main",
            "tsp_surveillance_trajectory = scoutiq_core.tsp_surveillance_trajectory:main",
        ],
    },
)
