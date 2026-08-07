from glob import glob

from setuptools import find_packages, setup

package_name = 'sensors_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muz',
    maintainer_email='muztahiddurjoy99@gmail.com',
    description='Turns raw MCU sensor topics into higher level ROS2 messages (wheel odometry).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'encoder_odometry = sensors_processing.encoder_odometry:main',
            'disparity_extender = sensors_processing.disparity_extender:main'
        ],
    },
)
