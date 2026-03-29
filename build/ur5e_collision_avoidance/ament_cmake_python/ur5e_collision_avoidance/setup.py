from setuptools import find_packages
from setuptools import setup

setup(
    name='ur5e_collision_avoidance',
    version='1.0.0',
    packages=find_packages(
        include=('ur5e_collision_avoidance', 'ur5e_collision_avoidance.*')),
)
