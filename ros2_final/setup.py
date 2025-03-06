from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanawat',
    maintainer_email='thanawat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_robot = ros2_final.control_robot:main',
            'stop_robot = ros2_final.stop_robot:main',
            'obstacle_detection = ros2_final.obstacle_detection:main',
            'show_msg = ros2_final.show_msg:main',
            'test_scan = ros2_final.test_scan:main',
        ],
    },
)
