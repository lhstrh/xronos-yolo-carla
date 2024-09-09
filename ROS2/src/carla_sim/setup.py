from setuptools import find_packages, setup

package_name = 'carla_sim'

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
    maintainer='depetrol',
    maintainer_email='depetrol@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla = carla_sim.carla_simulator:main',
            'agent = carla_sim.ppo_agent:main',
            'yolo = carla_sim.yolo_agent:main',
            'fusion = carla_sim.fusion:main',
            'carla_shm = carla_sim.carla_simulator_shm:main',
            'agent_shm = carla_sim.ppo_agent_shm:main',
            'yolo_shm = carla_sim.yolo_agent_shm:main',
            'fusion_shm = carla_sim.fusion_shm:main'
        ],
    },
)
