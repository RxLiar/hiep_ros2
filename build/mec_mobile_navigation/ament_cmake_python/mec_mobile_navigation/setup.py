from setuptools import find_packages
from setuptools import setup

setup(
    name='mec_mobile_navigation',
    version='0.0.0',
    packages=find_packages(
        include=('mec_mobile_navigation', 'mec_mobile_navigation.*')),
)
