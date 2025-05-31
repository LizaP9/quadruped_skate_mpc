from setuptools import setup
import os
from glob import glob

package_name = 'a1_inverse_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aida-user',
    maintainer_email='silverfrost.club@gmail.com',
    description='Inverse kinematics algorithms for Unitree A1 quadruped robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinematics = a1_inverse_kinematics.inverse_kinematics:main',
            'test_inverse_kinematics = a1_inverse_kinematics.test_inverse_kinematics:main',
            'visualize_ik_tests = a1_inverse_kinematics.visualize_ik_tests:main',
        ],
    },
)