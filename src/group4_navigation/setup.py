from glob import glob

from setuptools import find_packages, setup

package_name = 'group4_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mina',
    maintainer_email='robot@todo.todo',
    description='Nav2 launch + parameters for robot_4 on the live slam_toolbox map (round 2).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_goal = group4_navigation.send_goal:main',
            'odom_relay = group4_navigation.odom_relay:main',
        ],
    },
)
