from setuptools import find_packages, setup

package_name = 'face_recognition_pkg'

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
    maintainer='juno',
    maintainer_email='juno@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_recognition_client_node = face_recognition_pkg.face_recognition_client:main',
            'face_db_matcher_node = face_recognition_pkg.face_recognition_server:main'
        ],
    },
)
