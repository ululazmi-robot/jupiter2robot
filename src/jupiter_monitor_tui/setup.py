from setuptools import setup

package_name = 'jupiter_monitor_tui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jupiter Project',
    maintainer_email='jupiter@example.com',
    description='Textual dashboard for the Jupiter 2 monitor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = jupiter_monitor_tui.app:main',
            # The access panel: monitor + per-subsystem control.
            'panel = jupiter_monitor_tui.panel:main',
        ],
    },
)
