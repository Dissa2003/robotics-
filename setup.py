from setuptools import setup
from glob import glob
import os

package_name = 'ie4060_maze'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'dqn'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm') + glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='IE4060 Student Group',
    maintainer_email='student@example.com',
    description='TurtleBot3 Burger Gazebo maze, SLAM, Nav2 and DQN baseline',
    license='Apache-2.0',
    entry_points={'console_scripts': ['goal_metrics = ie4060_maze.goal_metrics:main']},
)
