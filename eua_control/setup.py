from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    package_dir={'': 'src'},
    packages=['eua_control', 'dynamixel'],
    scripts=['scripts/controller', 'scripts/center_servos']
)

setup(**d)
