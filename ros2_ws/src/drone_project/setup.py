from setuptools import find_packages, setup
import os
from glob import glob

# 1. Definimos el nombre correcto
package_name = 'drone_project'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jorge Castro',
    maintainer_email='tu_email@ejemplo.com',
    description='Puente entre ArduPilot y ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 2. CORRECCIÓN CRÍTICA AQUÍ:
            # Cambiamos 'drone_bridge' por 'drone_project' para que encuentre el archivo
            'interface_node = drone_project.interface_node:main',
            'mission_control_node = drone_project.mission_planner_node:main',
            'isaac_ardu_bridge = drone_project.bridge_ardupilot:main',
            'isaac_json_backend = drone_project.isaac_json_backend:main',
        ],
    },
)
