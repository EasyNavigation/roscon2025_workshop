from setuptools import setup

package_name = 'easynav_patrolling_behavior_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/patrolling.launch.py']),
        ('share/' + package_name + '/config', ['config/patrolling_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fmrico',
    maintainer_email='fmrico@gmail.com',
    description='Python reimplementation of the EasyNavigation Patrolling Behavior.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrolling_node = easynav_patrolling_behavior_py.patrolling_node:main',
        ],
    },
)
