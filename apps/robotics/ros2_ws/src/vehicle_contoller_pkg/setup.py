from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vehicle_contoller_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c102',
    maintainer_email='c102@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_controller_node = vehicle_contoller_pkg.drive_controller:main',
            'food_bay_controller_node = vehicle_contoller_pkg.food_bay_controller:main',
            'input_drive_teleop = vehicle_contoller_pkg.keyboard_controller:main',
            'pure_pursuit_controller_node = vehicle_contoller_pkg.pure_pursuit_controller:main',
        ]
    },
)
