import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moveit_task_constructor_kinova'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kedar Rajpathak',
    maintainer_email='kedar.rapa@gmail.com',
    description='MTC demo for kinova robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinova_pickplace = moveit_task_constructor_kinova.kinova_pickplace',
        ],
    },
)
