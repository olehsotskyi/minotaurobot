from setuptools import find_packages, setup

package_name = 'odometry_ir_reader'

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
            'odometry_ir_subscriber = odometry_ir_reader.odometry_ir_subscriber:main',
            'bumper_sensor_reader = odometry_ir_reader.bumper_sensor_reader:main',
        ],
    },
)
