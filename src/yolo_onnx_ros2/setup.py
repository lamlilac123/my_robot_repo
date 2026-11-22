from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_onnx_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lamlilac123',
    maintainer_email='lamlilac123@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
        entry_points={
        'console_scripts': [
            'AI_inference = yolo_onnx_ros2.AI_inference:main',
            'serial_writer = yolo_onnx_ros2.serial_writer:main',
        ],
    },

)
