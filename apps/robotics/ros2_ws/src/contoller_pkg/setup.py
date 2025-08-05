from setuptools import find_packages, setup

package_name = 'contoller_pkg'

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
    maintainer='c102',
    maintainer_email='c102@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = contoller_pkg.controller:main',
            'dc_motor_node = contoller_pkg.dc_motor:main',
            'servo_motor_node = contoller_pkg.servo_motor:main',
        ]
    },
)
