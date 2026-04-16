from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chethan_ms',
    maintainer_email='mschethan.26@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'patrol_node = drone_sim.patrol_node:main',
            'detection_node = drone_sim.detection_node:main',
            'mavros_control = drone_sim.mavros_control:main',
        ],
    },
)
