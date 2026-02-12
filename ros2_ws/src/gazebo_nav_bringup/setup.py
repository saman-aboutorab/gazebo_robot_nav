import os
from glob import glob
from setuptools import setup

package_name = 'gazebo_nav_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saman-aboutorab',
    maintainer_email='saman.aboutorab@gmail.com',
    description='Launch files and configs for Gazebo SLAM and Nav2 autonomous navigation with Turtlebot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
