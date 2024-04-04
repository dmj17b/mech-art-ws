from setuptools import setup

package_name = 'big_brother'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/big_brother_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Boylan',
    maintainer_email='jboylan@fsu.edu',
    description='Big Brother ROS Application',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_feed_node = big_brother.camera_feed_node:main',
            'camera_calibration_node = big_brother.camera_calibration_node:main',
            'web_image_node = big_brother.web_image_node:main',
        ],
    },
)
