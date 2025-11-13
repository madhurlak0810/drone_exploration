from setuptools import find_packages, setup

package_name = 'drone_rl'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py', 'launch/test_launch.py']),
        ('share/' + package_name + '/config', ['config/drone_config.yaml', 'config/drone_exploration.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/drone.urdf.xacro', 'urdf/drone_simple.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/exploration_world.world']),
        ('share/' + package_name + '/scripts', ['scripts/quick_start.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone_rl',
    maintainer_email='user@example.com',
    description='Autonomous drone exploration using reinforcement learning with RTAB-Map SLAM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'q_learning_agent = drone_rl.q_learning_agent:main',
            'environment_interface = drone_rl.environment_interface:main',
            'exploration_metrics = drone_rl.exploration_metrics:main',
        ],
    },
)