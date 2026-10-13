from glob import glob

from setuptools import find_packages, setup

package_name = 'bug2_create3'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='freek',
    maintainer_email='franklinkreleger@gmail.com',
    description='Bug2 explore run for the Create 3: start -> goal -> start (round 1).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bug2_maze = bug2_create3.bug2_node:main'
        ],
    },
)
