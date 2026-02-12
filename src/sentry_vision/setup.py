import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sentry_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=[
        'setuptools',
        'pupil-apriltags',
        'opencv-python',
        'numpy',
    ],
    zip_safe=True,
    maintainer='hamza',
    maintainer_email='hamzaighumman@gmail.com',
    description='AprilTag detection and pose estimation for sentry robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = sentry_vision.apriltag_detector:main',
        ],
    },
)
