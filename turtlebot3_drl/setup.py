import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_drl'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage1.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage2.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage3.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage4.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage5.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage6.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage7.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage8.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage9.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage10.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'FBE_world_1.launch.py'))),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Gilbert', 'Ryan Shim'],
    author_email=['kkjong@robotis.com', 'jhshim@robotis.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'DRL for TurtleBot3.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'environment = turtlebot3_drl.drl_environment.drl_environment:main',
            'fbe_environment = turtlebot3_drl.drl_environment.drl_environment_fbe:main',
            'real_environment = turtlebot3_drl.drl_environment.drl_environment_real:main',
            'gazebo_goals = turtlebot3_drl.drl_gazebo.drl_gazebo:main',
            'gazebo_fbe = turtlebot3_drl.drl_gazebo.drl_gazebo_fbe:main',
            'train_agent = turtlebot3_drl.drl_agent.drl_agent:main_train',
            'train_MQ_agent = turtlebot3_drl.drl_agent.MQ_drl_agent:main_train',
            'test_MQ_agent = turtlebot3_drl.drl_agent.MQ_drl_agent:main_train',
            'train_fbe_agent = turtlebot3_drl.drl_agent.drl_agent_fbe:main_train',
            'test_agent = turtlebot3_drl.drl_agent.drl_agent:main_test',
            'real_agent = turtlebot3_drl.drl_agent.drl_agent:main_real',
            'real_goal = turtlebot3_drl.drl_gazebo.real_goal:main',
            'gdae = turtlebot3_drl.gdae.gdae:main',
            'teleop_twist_keyboard = turtlebot3_drl.teleop_twist_keyboard.teleop_twist_keyboard:main',
            'frontier_exploration = turtlebot3_drl.frontier_exploration.frontier:main',
            'control = autonomous_exploration.control:main',
            'frontier_detector = turtlebot3_drl.learned_frontier_detector.learned_frontier_detector:main',
        ],
    },
)
