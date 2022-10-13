from setuptools import setup
import os
from glob import glob

package_name = 'om_fleet'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('*.yaml')),
        (os.path.join('share', package_name), glob('*.json'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rajaram',
    maintainer_email='rajaram@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_template.fleet_adapter:main',
            'fleet_manager=fleet_adapter_template.fleet_manager:main'
        ],
    },
)
