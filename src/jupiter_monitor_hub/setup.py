from setuptools import setup
import glob

package_name = 'jupiter_monitor_hub'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/drivers.yaml']),
        # glob, not a literal list: a missing launch dir must not break the
        # build (the same empty-directory trap that broke jupiterobot2_voice).
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Jupiter Project',
    maintainer_email='jupiter@example.com',
    description='Jupiter 2 driver manifest, hardware probe and health publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hub = jupiter_monitor_hub.hub_node:main',
            'manifest_check = jupiter_monitor_hub.manifest_check:main',
        ],
    },
)
