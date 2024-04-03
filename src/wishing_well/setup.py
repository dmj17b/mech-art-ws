from setuptools import find_packages, setup

package_name = 'wishing_well'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/wishing_well_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Boylan',
    maintainer_email='jboylan@fsu.edu',
    description='Wishing Well ROS Application',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_recording_node = wishing_well.audio_recording_node:main',
            'speech2text_node = wishing_well.speech2text_node:main',
            'wish_extraction_node = wishing_well.wish_extraction_node:main',
            'prompt_generation_node = wishing_well.prompt_generation_node:main',
            'image_generation_node = wishing_well.image_generation_node:main',
            'image_display_node = wishing_well.image_display_node:main',
            'image_save_node = wishing_well.image_save_node:main',
        ],
    },
)
