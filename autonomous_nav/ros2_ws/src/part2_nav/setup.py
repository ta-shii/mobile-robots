from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'part2_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUT4508 Group',
    maintainer_email='student@uwa.edu.au',
    description='Part 2 basic navigation for Pioneer 3-AT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_controller = part2_nav.gamepad_controller:main',
            'waypoint_navigator = part2_nav.waypoint_navigator:main',
            'cone_detector      = part2_nav.cone_detector:main',
            'cone_weaver        = part2_nav.cone_weaver:main',
            'object_detector    = part2_nav.object_detector:main',
            'journey_summary    = part2_nav.journey_summary:main',
            'lidar_safety       = part2_nav.lidar_safety:main',
        ],
    },
)
