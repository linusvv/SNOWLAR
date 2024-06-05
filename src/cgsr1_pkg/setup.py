from setuptools import find_packages, setup

package_name = 'cgsr1_pkg'

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
    maintainer='snowlar',
    maintainer_email='snowlar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'joy_steering = cgsr1_pkg.joytobase:main',
            'tank_motion = cgsr1_pkg.basecontrol:main',
            'imu_steering = cgsr1_pkg.imutobase:main',
            'web_interface = cgsr1_pkg.web_interface:main',
            'dummy_values = cgsr1_pkg.simple_publisher:main',
        ],
    },
)
