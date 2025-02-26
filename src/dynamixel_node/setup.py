from setuptools import find_packages, setup

package_name = 'dynamixel_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antorresca',
    maintainer_email='antorresca@todo.todo',
    description='Nodo de comunicacion con motores Dynamixel.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_node = dynamixel_node.dynamixel_node:main'
        ],
    },
)
