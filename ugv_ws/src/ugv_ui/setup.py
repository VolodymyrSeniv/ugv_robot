import os
from glob import glob
from setuptools import setup

package_name = 'ugv_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='volodymyr',
    maintainer_email='volodymyr.seniv@usepepper.com',
    description='Operator web UI for the UGV',
    license='MIT',
    entry_points={'console_scripts': ['robot_pose_pub = ugv_ui.robot_pose_pub:main']},
)
