from glob import glob

from setuptools import setup

package_name = 'turtlebot3_home_service_challenge_core'

setup(
    name=package_name,
    version='2.0.0',
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
            'home_service_challenge_core = \
                turtlebot3_home_service_challenge_core.home_service_challenge_core:main',
        ],
    },
)
