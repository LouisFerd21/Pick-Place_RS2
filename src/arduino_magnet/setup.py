from setuptools import setup

package_name = 'arduino_magnet'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'arduino_magnet.arduino_magnet_node',
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 Arduino electromagnet control',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'arduino_magnet_node = arduino_magnet.arduino_magnet_node:main',
        ],
    },
)

