from setuptools import find_packages, setup
import glob

package_name = 'simplified_dro'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/ui", glob.glob('ui/*.kv')),
        ('share/' + package_name + '/ui/images', glob.glob('ui/images/*')),
    ],
    install_requires=['setuptools','kivy==2.2.1'],
    zip_safe=True,
    maintainer='Trent Shumay',
    maintainer_email='trent@iotdesignshop.com',
    description='DRO (Digital Read Out) for Simplified Robotics and Interbotix Robot Arms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dro = simplified_dro.dro:main',
        ],
    },
)
