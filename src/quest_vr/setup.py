import os
from glob import glob

from setuptools import setup

package_name = 'quest_vr'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=['quest_reader'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='Read Quest VR glasses pose and publish PoseStamped for dual-arm servo tracking.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'quest_reader = quest_reader:main',
        ],
    },
)
