from setuptools import find_packages, setup

package_name = 'p3at'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tashi',
    maintainer_email='tashichimilhamo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_avoid = p3at.simple_avoid:main',
            'waypoint_controller = p3at.waypoint_controller:main',
            'multi_waypoint_controller = p3at.multi_waypoint_controller:main',
            'obstacle_aware_waypoint_controller = p3at.obstacle_aware_waypoint_controller:main',
            'path_recorder = p3at.path_recorder:main',
        ],
    },
)
