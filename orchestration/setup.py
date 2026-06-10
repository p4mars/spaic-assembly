import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'orchestration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frank Zillmann',
    maintainer_email='frank.zillmann@tum.de',
    description='High-level orchestrator/planner/executor for the tetris-assembly pipeline.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'orchestrator = orchestration.orchestrator:main',
        ],
    },
)
