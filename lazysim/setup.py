import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lazysim'

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
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Gorur Gari',
    maintainer_email='muztahid.appbaksho@gmail.com',
    description='Ignition Gazebo Fortress simulation of the Gorur Gari 2026 '
                'WRO Future Engineers car.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lazybridge = lazysim.lazybridge:main',
            'track_maker = lazysim.track_maker:main',
        ],
    },
)
