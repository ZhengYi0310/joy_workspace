## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['recorder'],
    package_dir={'':'src'},
    scripts=['scripts/BTAndJSRecorder.py', 'scripts/Server_Client.py'],
    requires=['rospy','rosjson_time', 'message_filters']
)

setup(**setup_args)
