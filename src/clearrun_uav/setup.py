from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'clearrun_uav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Hanzalah Javed',
    maintainer_email='hanzalah@example.com',
    description='UAV package for Clear-Run FOD detection system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = clearrun_uav.detection_node:main',
            'visual_servo = clearrun_uav.visual_servo:main',
            'mavros_interface = clearrun_uav.mavros_interface:main',
        ],
    },
)
