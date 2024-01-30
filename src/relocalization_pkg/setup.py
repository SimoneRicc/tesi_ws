from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'relocalization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'reloc_test'), glob('reloc_test/*.jpg')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nome nodo = pacchetto.nome_file_python:nome_funzione_main
            'acquire_query_node = relocalization_pkg.acquire_query_node:main',
            'pose_estimation_node = relocalization_pkg.pose_estimation_node:main',
            'reloc_eval_node = relocalization_pkg.reloc_eval_node:main',
            'show_poses_node = relocalization_pkg.show_poses_node:main',
            'converge_to_pose_node = relocalization_pkg.converge_to_pose_node:main',
        ],
    },
)
