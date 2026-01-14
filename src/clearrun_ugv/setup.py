from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'clearrun_ugv'

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
        # Map files
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Hanzalah Javed',
    maintainer_email='hanzalah@example.com',
    description='UGV package for Clear-Run FOD retrieval system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fod_retriever = clearrun_ugv.fod_retriever:main',
            'scoop_controller = clearrun_ugv.scoop_controller:main',
            'navigation_client = clearrun_ugv.navigation_client:main',
        ],
    },
)
