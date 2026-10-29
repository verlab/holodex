import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'holodex_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alisson',
    maintainer_email='alissonparizoto@gmail.com',
    description='Pacote central de integração e orquestração do robô Holodex Orange',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holodex_orchestrator = holodex_bringup.orchestrator:main',
            'benchmark_system = holodex_bringup.benchmark:main',
            'motion_supervisor = holodex_bringup.motion_supervisor:main',
        ],
    },
)
