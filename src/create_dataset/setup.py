from setuptools import find_packages, setup

package_name = 'create_dataset'

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
    maintainer='simone',
    maintainer_email='simone@todo.todo',
    description='Create dataset of RGB images of the environment for Visual Place Recognition',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_capture_node = create_dataset.image_capture:main',
            'reloc_capture_node = create_dataset.reloc_capture:main',
        ],
    },
)
