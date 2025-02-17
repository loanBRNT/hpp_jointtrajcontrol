from setuptools import find_packages, setup
from glob import glob

package_name = 'hpp_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/robots', glob('robots/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loan',
    maintainer_email='l.bernat@sileane.com',
    description='Simple Package to use hpp with the ros2_control JointTrajectoryController',
    license='Appache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solving_hpp = hpp_control.solving_hpp:main',
            'sending_hpp = hpp_control.sending_hpp:main'
        ],
    },
)
