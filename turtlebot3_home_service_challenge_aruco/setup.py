from glob import glob

from setuptools import setup

package_name = 'turtlebot3_home_service_challenge_aruco'

setup(
    name=package_name,
    version='1.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_parking = turtlebot3_home_service_challenge_aruco.aruco_parking:main',
            'aruco_tracker = turtlebot3_home_service_challenge_aruco.aruco_tracker:main',
        ],
    },
)
