from setuptools import find_packages, setup

package_name = 'roomwatch_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roomwatch',
    maintainer_email='user@todo.todo',
    description='Core utilities, trajectory processing, and navigation nodes for roomwatch',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_data = roomwatch_core.save_data:main',
            'listener = roomwatch_core.listener:main',
            'navigate = roomwatch_core.navigate:main',
            'publish_traj = roomwatch_core.publish_traj:main',
            'extract_robot_trajectory = roomwatch_core.extract_robot_trajectory:main',
            'tsp_surveillance_trajectory = roomwatch_core.tsp_surveillance_trajectory:main',
        ],
    },
)
