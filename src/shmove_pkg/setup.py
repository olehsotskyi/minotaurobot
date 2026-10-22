from setuptools import find_packages, setup

package_name = 'shmove_pkg'

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
            'ir_sensors = shmove_pkg.ir_sensors:main',
            'go_to_goal = shmove_pkg.go_to_goal:main',
            'shmove_to_goal = shmove_pkg.shmove_to_goal:main',
            'wall_follower = shmove_pkg.wall_follower:main',
            'mover = shmove_pkg.mover:main',
            'algorithm_no_classes = shmove_pkg.algorithm_no_classes:main',
            'algorithm_classes = shmove_pkg.algorithm_classes:main',
            'algorithm_extra_states = shmove_pkg.algorithm_extra_states:main',
            'wall_shmover = shmove_pkg.wall_shmover:main',
            'starting_pose = shmove_pkg.starting_pose:main',
            'test = shmove_pkg.test:main',
            'test2 = shmove_pkg.test2:main',
        ],
    },
)
