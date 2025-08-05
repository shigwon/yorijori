from setuptools import find_packages, setup

package_name = 'video_demo_pkg'

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
    maintainer_email='juno980220@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_demo_publisher_node = video_demo_pkg.video_demo_publisher:main',
            'video_demo_subscriber_node = video_demo_pkg.video_demo_subscriber:main'
        ],
    },
)
