from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'turtle_circle'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Drive turtlesim in a circle with configurable period/amplitude, '
                'record a rosbag2 and analyze it.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'circle_node = turtle_circle.circle_node:main',
            'record_bag = turtle_circle.record_bag:main',
            'analyze_bag = turtle_circle.analyze_bag:main',
        ],
    },
)
