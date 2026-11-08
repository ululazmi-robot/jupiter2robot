from glob import glob
from setuptools import setup

package_name = 'jupiterobot2_head_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jupiter Project',
    maintainer_email='jupiter@example.com',
    description='ROS 2 bringup for the Jupiter 2 head pan joint (AX-12A)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_node = jupiterobot2_head_bringup.head_node:main',
        ],
    },
)
