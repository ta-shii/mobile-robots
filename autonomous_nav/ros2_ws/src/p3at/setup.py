from glob import glob
from setuptools import find_packages, setup

package_name = 'p3at'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*.*')),
        ('share/' + package_name + '/meshes/p3at_meshes', glob('meshes/p3at_meshes/*.dae')),
        ('share/' + package_name + '/meshes/p3at_meshes', glob('meshes/p3at_meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tashi',
    maintainer_email='tashichimilhamo@gmail.com',
    description='P3AT Pioneer robot URDF and RViz config for Part 3',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
