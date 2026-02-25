from setuptools import setup
import os
from glob import glob

# MUST match your actual folder name
package_name = 'paper_implementation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include model files
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='CRPF Motion Planner Case 1',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # syntax: executable_name = package_folder.filename:function
            'paper_node = paper_implementation.collision_node:main',
        ],
    },
)