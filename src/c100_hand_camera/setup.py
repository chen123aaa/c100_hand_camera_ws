from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'c100_hand_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raybot',
    maintainer_email='raybot@example.com',
    description='ROS2 driver wrapper for the WHEELTEC C100 eye-in-hand USB camera.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'c100_hand_camera_node = c100_hand_camera.c100_hand_camera_node:main',
            'c100_detect_devices = c100_hand_camera.detect_devices:main',
            'c100_calibrate_camera = c100_hand_camera.calibrate_camera:main',
            'c100_detect_apriltag = c100_hand_camera.c100_apriltag_detector_node:main',
        ],
    },
)
