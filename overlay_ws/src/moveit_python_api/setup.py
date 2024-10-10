import os
from glob import glob
from setuptools import setup

package_name = 'moveit_python_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name), glob('launch/*.py')),
        # (os.path.join('share', package_name), glob('config/*.rviz')),
        # (os.path.join('share', package_name), glob('config/*.lua'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kedar Rajpathak',
    maintainer_email='kedar.rajpathak@wzl-iqs.rwth-aachen.de',
    description='This package contains code for using moveit with kinova gen3 7dof',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "moveit_python_api_node = moveit_python_api.motion_planning_python_api:main",
            "save_camera_images_node = moveit_python_api.save_camera_images:main",
        ],
    },
)
