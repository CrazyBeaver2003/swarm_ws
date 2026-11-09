from setuptools import find_packages, setup
from glob import glob

package_name = 'mesh_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
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
            'rtt_sensor = mesh_localization.sensor_node:main',
            'mesh_localizer = mesh_localization.localizer_node:main',

        ],
    },
)
