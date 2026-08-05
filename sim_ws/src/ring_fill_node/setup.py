from setuptools import setup

package_name = 'ring_fill_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lightning',
    maintainer_email='dev@lightning.local',
    description='Adds a ring field to Gazebo point clouds (16 lines, -15..15 deg)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ring_fill_node = ring_fill_node.ring_fill_node:main',
        ],
    },
)
