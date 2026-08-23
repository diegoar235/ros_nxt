from setuptools import find_packages
from setuptools import setup

setup(
    name='isaac_ros_common',
    version='3.2.5',
    packages=find_packages(
        include=('isaac_ros_common', 'isaac_ros_common.*')),
)
