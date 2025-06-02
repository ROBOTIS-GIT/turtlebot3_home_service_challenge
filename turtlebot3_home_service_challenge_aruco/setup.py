import glob

from setuptools import setup

package_name = 'turtlebot3_home_service_challenge_aruco'

setup(
    name=package_name,
    version='1.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=['ChanHyeong Lee'],
    author_email=['dddoggi1207@gmail.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Home Service Challenge for TurtleBot3.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_parking = turtlebot3_home_service_challenge_aruco.aruco_parking:main',
            'aruco_tracker = turtlebot3_home_service_challenge_aruco.aruco_tracker:main',
        ],
    },
)
