from setuptools import find_packages, setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'turtlesim'],
    zip_safe=True,
    maintainer='cloudycorn',
    maintainer_email='cloudycorn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_movement = turtle_controller.turtle_square:main',
            'pose_subscriber = turtle_controller.pose_subscriber:main',
        ],
    },
)
