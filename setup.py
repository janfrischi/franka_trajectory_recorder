from setuptools import setup

package_name = 'franka_trajectory_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package to record trajectories for the Franka Emika robot.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_recorder = franka_trajectory_recorder.trajectory_recorder:main',
            'trajectory_recorder_new = franka_trajectory_recorder.trajectory_recorder_new:main',
            'trajectory_playback = franka_trajectory_recorder.trajectory_playback:main',
            'joint_position_sender = franka_trajectory_recorder.joint_position_sender:main'
        ],
    },
)
