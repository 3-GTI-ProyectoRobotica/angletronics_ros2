from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'angletronics_service'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('test/*.py'))        

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
            'movement_server = angletronics_service.movement_server:main' #incluir
        ],
    },
)
