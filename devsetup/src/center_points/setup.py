# from setuptools import setup

# setup(
#     name='center_points',
#     version='0.0.0',
#     packages=['center_points'],
#     package_dir={'': ''}
# )

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    package_xml_path='package.xml',
    packages=['center_points'],
    package_dir={'': 'src'}
)

setup(**d)