from setuptools import setup
from glob import glob
import os 

package_name = 'hri_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    zip_safe=True,
    maintainer='Petr Vanc',
    maintainer_email='petr.vanc@cvut.cz',
    description='',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "action_executor = hri_manager.action_executor:action_executor",
            "action_executor_dry_run = hri_manager.action_executor:action_executor_dry_run",
            "link_gesture_to_action = hri_manager.link_gesture_to_action:main",
        ],
    },
)
