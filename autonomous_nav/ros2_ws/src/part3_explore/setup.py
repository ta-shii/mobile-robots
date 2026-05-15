from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'part3_explore'

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
    maintainer='AUTO4508 Group 9',
    maintainer_email='xxxx@student.uwa.edu.au',
    description='Part 3 – Mapping and discovery for Pioneer 3-AT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = part3_explore.mission_manager:main',
            'safety_monitor  = part3_explore.safety_monitor:main',
            'color_detector   = part3_explore.color_detector:main',
        ],
    },
)