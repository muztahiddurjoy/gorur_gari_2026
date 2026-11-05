import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fastrobox_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Gorur Gari',
    maintainer_email='muztahid.appbaksho@gmail.com',
    description='Simulation of the BUBT FastRobox 1.0 Smart Navigation Challenge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
