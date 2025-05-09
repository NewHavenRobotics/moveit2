from setuptools import setup, find_packages  # Import find_packages
import os
from glob import glob

package_name = 'moveit_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # Automatically find all submodules
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='MoveIt Servo package with joy repeater functionality.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_repeater = moveit_servo.joy_repeater:main',  # Register joy_repeater.py as an executable
        ],
    },
)