from setuptools import setup
import os
import glob

package_name = 'hive_imu'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'hive_imu/imusensor', 'hive_imu/imusensor/filters', 'hive_imu/imusensor/MPU9250'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nils Schulte',
    maintainer_email='git@nilsschulte.de',
    description='Driver package for the MPU9250 sensor package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [        
            'mpu9250 = hive_imu.mpu9250:main',
            'tf_ned_to_enu = hive_imu.tf_ned_to_enu:main',
        ],
    },
)
