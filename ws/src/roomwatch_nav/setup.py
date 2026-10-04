import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roomwatch_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roomwatch',
    maintainer_email='dev@roomwatch.org',
    description='Nav2 and SLAM setup for roomwatch mobile manipulator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_initial_pose = roomwatch_nav.pub_initial_pose:main',
            'save_map = roomwatch_nav.save_map:main',
        ],
    },
)
