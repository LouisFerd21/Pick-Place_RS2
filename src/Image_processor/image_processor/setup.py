from setuptools import setup
import os

package_name = 'image_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='easha',
    maintainer_email='easha.mohammedghouseuddin@student.uts.edu.au',
    description='Image processor GUI with ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_gui = image_processor.run_gui:main_ros',
            'pixel_publisher = image_processor.pixel_publisher:main',
        ],
    },
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files here
        ('share/' + package_name + '/launch', ['image_processor/launch.py']),
    ],
)

