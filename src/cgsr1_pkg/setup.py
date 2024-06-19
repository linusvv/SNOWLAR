from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cgsr1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'joytobase = cgsr1_pkg.joytobase:main',
            'basecontrol = cgsr1_pkg.basecontrol:main',
            'imutobase = cgsr1_pkg.imutobase:main',
            'web_interface = cgsr1_pkg.web_interface:main',
            'dummy_publisher = cgsr1_pkg.dummy_publisher:main',
            'winchcontrol = cgsr1_pkg.winchcontrol:main',
            'computation = cgsr1_pkg.computation:main',
            'imucontrol = cgsr1_pkg.imucontrol:main',
            'calibration = cgsr1_pkg.calibration_node:main',
            'automation = cgsr1_pkg.automation:main',
            'straight_motion = cgsr1_pkg.straight_motion:main',
            'winch_feedback = cgsr1_pkg.winch_feedback:main',
            'linus_automation = cgsr1_pkg.linus_automation:main',
            'brushtest = cgsr1_pkg.brushtest:main',
        ],
    },
)