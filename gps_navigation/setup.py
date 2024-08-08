from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='makarand',
    maintainer_email='makarand.ericrobotics@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = gps_navigation.logged_waypoint_follower:main',
            'interactive_waypoint_follower = gps_navigation.interactive_waypoint_follower:main',
            'gps_waypoint_logger = gps_navigation.gps_waypoint_logger:main',
            'waypoint_publisher = gps_navigation.waypoint_publisher:main',
            'waypoint_input = gps_navigation.waypoint_input:main',
            'simple_navigation = gps_navigation.simple_navigation:main',
            'chatgpt_waypoint = gps_navigation.chatgpt_waypoint:main',  
        ],
    },
)

