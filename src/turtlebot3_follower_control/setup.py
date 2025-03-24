from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'turtlebot3_follower_control'

setup(
    name=package_name,
	version='0.0.1',
	packages=[package_name],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='your_name',
	maintainer_email='your_email@example.com',
	description='TurtleBot3 Follower Control Node',
	license='Apache License 2.0',
	tests_require=['pytest'],
	entry_points={
    	'console_scripts': [
        	'follower_control = turtlebot3_follower_control.follower_control:main',
    	],
	},  

)
