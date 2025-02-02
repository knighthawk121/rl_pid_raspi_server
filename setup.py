from setuptools import setup

package_name = 'rl_pid_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arvindh_Ramesh',
    maintainer_email='arvindh.r@outlook.com',

    description='ROS2 PID tuning action server for motor control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_server = rl_pid_server.pid_server_node:main',
            'rl_pid_server = rl_pid_server.rl_pid_server:main'
        ],
    },
)