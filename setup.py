## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKING INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_disutils_setup

#fetch values from package.xml
setup_args = generate_distutils_setup(
	packages=['pid_controller'],
	package_dir={' ' : 'include'},
)
setup(**setup_args)

