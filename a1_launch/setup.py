from setuptools import setup
import os
from glob import glob

package_name = 'a1_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aida-user',
    maintainer_email='silverfrost.club@gmail.com',
    description='Launch files for Unitree A1 quadruped robot simulation and control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a1_mujoco_viewer = a1_launch.a1_mujoco_viewer:main',
        ],
    },
)