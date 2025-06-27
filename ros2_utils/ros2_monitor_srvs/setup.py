from setuptools import setup

package_name = 'ros2_monitor_srvs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/GetTopicInfo.srv']),
    ],
    install_requires=['setuptools', 'rosidl_default_generators'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Custom service definitions for ROS2 Monitor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
) 