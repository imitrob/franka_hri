from setuptools import setup
from glob import glob
import os 

package_name = 'llm_merger'

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
            "llm_merger = llm_merger.llm_model:main",
        ],
    },
)
