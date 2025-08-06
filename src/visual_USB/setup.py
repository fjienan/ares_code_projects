from setuptools import find_packages, setup

package_name = 'visual_USB'

setup(
    name='visual-USB',  # Package name with hyphen as referenced in error
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'cv_bridge',
        'opencv-python',
        'numpy',
        'ultralytics'
    ],
    zip_safe=True,
    maintainer='ares',
    maintainer_email='fjienan@163.com',
    description='Visual USB package for camera-based positioning',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video = visual_USB.video:main',
        ],
    },
)