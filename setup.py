import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bento_teleop'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'parameter'), glob(os.path.join('parameter', '*.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Pelz',
    maintainer_email='bento.robotics@gmail.com',
    description='Teleoperation for bento robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = bento_teleop.bento_teleop:main',
            'arm_teleop_node = bento_teleop.bento_arm_teleop:main',
        ],
    },
)
