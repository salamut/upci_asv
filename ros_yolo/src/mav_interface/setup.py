from setuptools import find_packages, setup

package_name = 'mav_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salamut',
    maintainer_email='salamut@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mav_monitor = mav_interface.mav_monitor_node:main',
            'mission_master = mav_interface.mission_master_node:main'
        ],
    },
)
