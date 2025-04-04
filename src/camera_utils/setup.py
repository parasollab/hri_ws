from setuptools import find_packages, setup

package_name = 'camera_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/record_demo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='courtney',
    maintainer_email='cmcbeth5287@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_camera = camera_utils.record_camera:main',
            'pointcloud_subscriber = camera_utils.pointcloud_subscriber:main'
        ],
    },
)
