from setuptools import setup


package_name = 'fsw_ros2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swhart',
    maintainer_email='swhart@traclabs.com',
    description='BRASH FSW to ROS2 Bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'fsw_ros2_bridge = fsw_ros2_bridge.fsw_ros2_bridge:main'
            ],
    },
)
