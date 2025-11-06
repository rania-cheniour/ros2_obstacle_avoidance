from setuptools import find_packages, setup

package_name = 'ros2_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', ['launch/avoider.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot_real.urdf']),
        ('share/' + package_name + '/meshes', ['meshes/rplidar_a2.dae']), 
        ('share/' + package_name + '/rviz', ['rviz/robot.rviz']),
        ('share/' + package_name + '/worlds', ['worlds/empty.sdf']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rania',
    maintainer_email='rania.cheniour@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fuzzy_avoider = ros2_obstacle_avoidance.fuzzy_obstacle_avoider:main',
        ],
    },
)
