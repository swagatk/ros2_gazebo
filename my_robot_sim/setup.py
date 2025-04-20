from setuptools import find_packages, setup

package_name = 'my_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/display.launch.py']),
        ('share/' + package_name + '/description',
            ['description/my_robot.sdf']),
        ('share/' + package_name + '/rviz',
            ['rviz/config.rviz']),
        ('share/' + package_name + '/world' , 
            ['world/my_world.sdf']),
        ('share/' + package_name + '/world' , 
            ['world/my_world_3.sdf']),
        ('share/' + package_name + '/config',
            ['config/bridge_config.yaml']),
        ('share/' + package_name + '/config', 
            ['config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swagat',
    maintainer_email='swagat.kumar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoid_obstacle = my_robot_sim.avoid_obstacle:main',
            'follow_wall = my_robot_sim.follow_wall:main',
        ],
    },
)
