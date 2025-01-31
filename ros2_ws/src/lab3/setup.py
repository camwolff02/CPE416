import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lab3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='cameron.wolff@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_to_pose = lab3.move_to_pose:main',
            'turn_to_angle = lab3.turn_to_angle:main',
            'draw_square = lab3.draw_square:main'
        ],
    },
)
