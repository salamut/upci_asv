from setuptools import setup
import os
from glob import glob

package_name = 'asv_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ⬇️ Tambahkan baris ini agar ROS2 membaca folder launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sabiq',
    maintainer_email='example@mail.com',
    description='ASV Launch Package',
    license='MIT',
    entry_points={
        'console_scripts': []
    },
)
