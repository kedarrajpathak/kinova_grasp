import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kinova_images'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires= ['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='Kedar Rajpathak',
    maintainer_email='kedar.rapa@gmail.com',
    description='Saves published images to a folder',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber = kinova_images.camera_subscriber:main',
        ],
    },
)
