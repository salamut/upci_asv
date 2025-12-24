import os
from setuptools import find_packages, setup

package_name = 'web_monitoring'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name+'/web', ['web_monitoring/index.html'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucky_fool',
    maintainer_email='zakyfauzi44@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_monitoring_node = web_monitoring.web_monitoring_node:main'
        ],
    },
)
