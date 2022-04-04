from setuptools import setup
import os
from glob import glob
package_name = 'angletronics_ros2_ruta'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juanc',
    maintainer_email='juancarloshr123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
        'action_server = angletronics_ros2_ruta.action_server:main',
        'action_client = angletronics_ros2_ruta.action_client:main' 
        ],
    },
)
