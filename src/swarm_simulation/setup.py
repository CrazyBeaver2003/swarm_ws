from setuptools import find_packages, setup

package_name = 'swarm_simulation'

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
    maintainer='tim',
    maintainer_email='timagadenov1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "gazebo_bridge = swarm_simulation.gazebo_bridge:main",
            'mesh_px4_bridge = swarm_simulation.mesh_px4_bridge:main',
            'mesh_startup = swarm_simulation.mesh_startup:main',
        ],
    },
)
