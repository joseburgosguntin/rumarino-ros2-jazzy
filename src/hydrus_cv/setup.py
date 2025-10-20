from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hydrus_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.20.0',
        'opencv-python>=4.5.0',
        'torch>=2.0.0',
        'ultralytics>=8.0.0',  
    ],
    zip_safe=True,
    maintainer='cesar',
    maintainer_email='cruiznavarro44@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_publisher = hydrus_cv.depth_publisher:main',
            'cv_publisher = hydrus_cv.cv_publisher:main',
            'map_visualizer = hydrus_cv.map_visualizer:main',
        ],
    },
)
