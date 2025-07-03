from setuptools import setup

package_name = 'udp_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=['udp_robot'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ares',
    maintainer_email='3398993264@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_robot_node = udp_robot.robot_udp_node:main',
        ],
    },
)
