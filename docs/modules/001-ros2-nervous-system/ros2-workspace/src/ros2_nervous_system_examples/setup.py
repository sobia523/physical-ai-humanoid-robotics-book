from setuptools import setup
import os
from glob import glob

package_name = 'ros2_nervous_system_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Examples for the ROS 2 Nervous System module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_publisher = ros2_nervous_system_examples.hello_publisher:main',
            'hello_subscriber = ros2_nervous_system_examples.hello_subscriber:main',
            'string_reversal_service = ros2_nervous_system_examples.string_reversal_service:main',
            'simple_publisher = ros2_nervous_system_examples.simple_publisher:main',
            'simple_subscriber = ros2_nervous_system_examples.simple_subscriber:main',
            'simple_service_server = ros2_nervous_system_examples.simple_service_server:main',
            'simple_service_client = ros2_nervous_system_examples.simple_service_client:main',
            'urdf_loader = ros2_nervous_system_examples.urdf_loader:main',
            'python_agent = ros2_nervous_system_examples.python_agent:main',
        ],
    },
)