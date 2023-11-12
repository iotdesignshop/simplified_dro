from setuptools import find_packages, setup
from glob import glob

package_name = 'simplified_robotics'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/ui", glob('ui/*.kv')),
        ('share/' + package_name + '/ui/images', glob('ui/images/*')),
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools','kivy==2.2.1'],
    zip_safe=True,
    maintainer='Trent Shumay',
    maintainer_email='trent@iotdesignshop.com',
    description='UI Control Package for Simplified Robotics and Interbotix Robot Arms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dro = simplified_robotics.dro:main',
            'control = simplified_robotics.control:main',
        ],
    },
)
