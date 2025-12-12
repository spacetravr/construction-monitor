import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'construction_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'models', 'robot1'), glob('models/robot1/*')),
        (os.path.join('share', package_name, 'models', 'robot2'), glob('models/robot2/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezis',
    maintainer_email='ezis@todo.todo',
    description='Construction Site Progress Monitoring with TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_explorer = construction_monitor.auto_explorer:main',
            'auto_explorer_ns = construction_monitor.auto_explorer_ns:main',
            'auto_explorer_zone = construction_monitor.auto_explorer_zone:main',
            'map_merger = construction_monitor.map_merger:main',
            'tf_relay = construction_monitor.tf_relay:main',
        ],
    },
)
