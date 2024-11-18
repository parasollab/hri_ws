from setuptools import setup

package_name = 'trajectory_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to write JointTrajectory messages to JSON',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_subscriber = trajectory_utils.trajectory_subscriber:main',
            'test_publisher = trajectory_utils.test_publisher:main',
            'test_state_publisher = trajectory_utils.test_state_publisher:main',
            'numpy_trajectory_subscriber = trajectory_utils.numpy_trajectory_subscriber:main',
        ],
    },
)
