import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='yassin.frh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors_drive = robot_control.motors_drive:main',
            'ultrasonic = robot_control.ultrasonic:main',
            'led_panel = robot_control.led_panel:main',
            'camera_servo_drive = robot_control.camera_servo_drive:main',
        ],
    },
)
