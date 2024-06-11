# my_hand_tracking_pkg/setup.py
from setuptools import setup

package_name = 'my_hand_tracking_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','opencv-python','mediapipe'],
    zip_safe=True,
    maintainer='Chloe Caut',
    maintainer_email='chloe.caut@gmail.com',
    description='Hand tracking package using MediaPipe and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hands_tracking_node = my_hand_tracking_pkg.hands_tracking_node:main',
        ],
    },
)
