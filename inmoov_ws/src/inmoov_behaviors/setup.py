from setuptools import find_packages, setup

package_name = 'inmoov_behaviors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Alonso Puig',
    maintainer_email='aalonsopuig@gmail.com',
    description='Behavior nodes for face tracking and recognition on the InMoov robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_tracking_behavior_node = inmoov_behaviors.face_tracking_behavior_node:main',
            'face_recognized_behavior_node = inmoov_behaviors.face_recognized_behavior_node:main',
        ],
    },
    include_package_data=True,
    package_data={
        'inmoov_behaviors': [
            'greetings_unknown.txt',
            'greetings_known.txt',
            'movements_known.yaml',
        ],
    },
)
