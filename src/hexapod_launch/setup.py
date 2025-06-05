from setuptools import find_packages, setup

package_name = 'hexapod_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',['launch/hexapod_launch.py'] ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antorresca',
    maintainer_email='antorresca@unal.edu.co',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
