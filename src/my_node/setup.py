from setuptools import find_packages, setup
#hence this too
import os

package_name = 'my_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #the line i added
        (os.path.join('share', package_name, 'launch'), ['launch/node_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cloudycorn',
    maintainer_email='cloudycorn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = my_node.publisher_node:main',
            'subscriber = my_node.subscriber_node:main',
            'hello_publisher = my_node.hello_pub:main',
            'hello_subscriber = my_node.hello_sub:main',
        ],
    },
)
