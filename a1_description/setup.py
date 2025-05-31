from setuptools import setup
import os
from glob import glob

package_name = 'a1_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('*.xml')),
        ('share/' + package_name, glob('*.png')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/assets', glob('assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aida-user',
    maintainer_email='silverfrost.club@gmail.com',
    description='Unitree A1 robot description files and URDF models',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)