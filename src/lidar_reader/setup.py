from setuptools import find_packages, setup

package_name = 'lidar_reader'

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
    maintainer='robot',
    maintainer_email='olehsotskyi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
       entry_points={
        'console_scripts': [
            'lidar_subscriber = lidar_reader.lidar_subscriber:main',
            'laser = lidar_reader.Laser:main',
            'laser_v2 = lidar_reader.Laser_v2:main',
            'ir_sensors = lidar_reader.ir_sensors:main',
            'shmove_to_goal = lidar_reader.shmove_to_goal:main',
            'wall_follower = lidar_reader.wall_follower:main',
            'algorithm_extra_states = lidar_reader.algorithm_extra_states:main',
            'wall_shmover = lidar_reader.wall_shmover:main',
            'starting_pose = lidar_reader.starting_pose:main',
        ],
    },  
)
