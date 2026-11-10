from setuptools import find_packages, setup

setup(
    name='swarm_evaluation', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/swarm_evaluation']),
        ('share/swarm_evaluation', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='tim', maintainer_email='tim@example.com',
    description='Passive flight recorder and time-aligned experiment metrics',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'evaluator = swarm_evaluation.node:main',
        'evaluate = swarm_evaluation.metrics:main',
    ]},
)
