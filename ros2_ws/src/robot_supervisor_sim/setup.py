from glob import glob

from setuptools import find_packages, setup


package_name = 'robot_supervisor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Fortune Olose',
    maintainer_email='fortuneolose@users.noreply.github.com',
    description='Synthetic ROS2-010 topology and deterministic fault injection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = robot_supervisor_sim.node:main',
        ],
    },
)
