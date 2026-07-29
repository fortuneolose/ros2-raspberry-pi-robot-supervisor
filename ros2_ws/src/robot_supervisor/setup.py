from glob import glob
from pathlib import Path

from setuptools import setup


package_name = 'robot_supervisor'
repository_root = Path('..') / '..' / '..'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'models'],
    package_dir={
        'models': str(repository_root / 'models'),
    },
    package_data={
        'models': [
            'parameters/synthetic_motor.json',
            'parameters/synthetic_sim_010.json',
        ]
    },
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
    description=(
        'Hardware-independent ROS 2 Jazzy middleware integration for SIM-010.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor_node = robot_supervisor.node:main',
            'generate_ros2_010_evidence = '
            'robot_supervisor.generate_evidence:main',
        ],
    },
)
