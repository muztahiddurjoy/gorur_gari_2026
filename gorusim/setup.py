import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gorusim'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Gorur Gari',
    maintainer_email='muztahid.appbaksho@gmail.com',
    description='Ignition Gazebo Fortress simulation of the Gorur Gari 2026 '
                'WRO Future Engineers car, built from its CAD and firmware. '
                'Track tooling adapted from Team LazyGo '
                '(https://github.com/A-N-M-Noor/LazyGo_WRO2025/); for testing '
                'purposes only, not part of the competition build.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_mcu = gorusim.sim_mcu:main',
            'track_maker = gorusim.track_maker:main',
        ],
    },
)
