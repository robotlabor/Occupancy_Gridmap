from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'occupancy_gridmap'

setup(
    name=package_name,
    version='0.4.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='@gmail.com',
    description=(
        'GeoTIFF OccupancyGrid generation and a graphical map, cone line, '
        'cone arc, coordinate polygon, and robot start editor for ROS 2.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ortho_gridmap_node = occupancy_gridmap.ortho_gridmap_node:main',
            'map_editor = occupancy_gridmap.map_editor:main',
        ],
    },
)
