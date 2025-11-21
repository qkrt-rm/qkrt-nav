from setuptools import find_packages, setup

package_name = 'qkrt_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='revereziyiwang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'imu_publisher = qkrt_imu.imu_publisher:main',
        'imu_republisher = qkrt_imu.imu_republisher:main',
        'imu_listener   = qkrt_imu.imu_listener:main',
    ],
},

)
