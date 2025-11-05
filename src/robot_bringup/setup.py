import os
from glob import glob
from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Register the package in the ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/robot_bringup/launch', glob('launch/*.launch.py')),
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install rviz configuration files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Install urdf files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Robot bringup package with launch and config files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any python node scripts here e.g. 'node_name = robot_bringup.node_script:main',
        ],
    },
)
