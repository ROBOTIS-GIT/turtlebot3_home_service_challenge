from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_home_service_challenge_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detect = turtlebot3_home_service_challenge_aruco.aruco_detect:main',
            'aruco_parking = turtlebot3_home_service_challenge_aruco.aruco_parking:main',
        ],
    },
)
