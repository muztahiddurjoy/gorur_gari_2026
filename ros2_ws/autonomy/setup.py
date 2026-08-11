from setuptools import find_packages, setup

package_name = 'autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muz',
    maintainer_email='muztahiddurjoy99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goto_controller = autonomy.goto_controller:main',
            'disparity_extender = autonomy.disparity_extender:main',
            'open_round_run = autonomy.open_round_run:main',
            'vision_node = autonomy.vision_node:main',
            'lap_counter = autonomy.lap_counter:main',
            'tune_hsv = autonomy.tune_hsv:main',
            'testing = autonomy.custom_disparity_extender:main',
            'custom_disparity_extender = autonomy.custom_disparity_extender:main',
            'camera_push_detector = autonomy.camera_push_detector:main',
        ],
    },
)
