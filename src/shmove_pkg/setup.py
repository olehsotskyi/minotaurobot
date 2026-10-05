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
            'go_to_goal = shmove_pkg.go_to_goal:main',
            'wall_follower = shmove_pkg.wall_follower:main',
            'mover = shmove_pkg.mover:main',
            'shmoving_algorithm_1 = shmove_pkg.shmoving_algorithm_1:main',
            'wall_shmover = shmove_pkg.wall_shmover:main',
            'test = shmove_pkg.test:main',
        ],
    },
)
