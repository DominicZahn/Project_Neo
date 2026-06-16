from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dodge_it_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Zahn',
    maintainer_email='zahndominic@web.de',
    description='Dodge_it develops multiple different algorithms to learn more about dodge motions of H1.',
    license='MIT',
    extras_require={},
    entry_points={
    'console_scripts': [
        'jackson = dodge_it_py.jackson.jackson:main',
        'bob = dodge_it_py.bob.bob:main',
        'john = dodge_it_py.john.john:main',
        'neo = dodge_it_py.neo.main:main'
    ]
}
)
