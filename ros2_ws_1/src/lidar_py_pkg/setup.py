from setuptools import find_packages, setup

package_name = 'lidar_py_pkg'

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
    maintainer='enes',
    maintainer_email='enes@todo.todo',
    description='Simple Lidar Python Node',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_data_node = lidar_py_pkg.lidar_data_node:main'
        ],
    },
)

