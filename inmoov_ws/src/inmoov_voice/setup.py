from setuptools import find_packages, setup

package_name = 'inmoov_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',                # ROS 2 Python client library
        'soundfile',            # WAV file reading
        'numpy',                # Audio array processing
        'sounddevice',          # Real-time audio playback
    ],
    zip_safe=True,
    maintainer='Alejandro Alonso Puig',
    maintainer_email='aalonsopuig@gmail.com',
    description='ROS 2 package for speech synthesis with Piper and jaw actuation for the InMoov robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_node = inmoov_voice.tts_node:main',
            'tts_jaw_node = inmoov_voice.tts_jaw_node:main',
        ],
    },
)
