from setuptools import find_packages
from setuptools import setup

setup(
    name='mini_pupper_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('mini_pupper_interfaces', 'mini_pupper_interfaces.*')),
)
