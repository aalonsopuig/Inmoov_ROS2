from setuptools import setup

package_name = 'xicro_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/xicro_nodes.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Alonso Puig',
    maintainer_email='todo@todo.com',
    description='ROS 2 package for XICRO communication nodes between ROS 2 and Arduino in the InMoov robot. Includes nodes for subsystem 1 (right arm) and subsystem 2 (left arm and head).',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xicro_node_subsys1_ID_1_arduino = xicro_nodes.xicro_node_subsys1_ID_1_arduino:main',
            'xicro_node_subsys2_ID_2_arduino = xicro_nodes.xicro_node_subsys2_ID_2_arduino:main',
        ],
    },
)
