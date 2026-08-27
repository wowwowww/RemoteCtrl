import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sr_amr_control'

setup(
  name=package_name,
  version='0.1.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (
      os.path.join('share', package_name, 'launch'),
      glob(os.path.join('launch', '*launch.[pxy]*')),
    ),
  ],
  install_requires=['setuptools', 'sros-sdk-py'],
  zip_safe=True,
  maintainer='zhangxiao',
  maintainer_email='zhangxiao@standard-robots.com',
  description='Standard Robots AMR control package',
  license='BSD-3-Clause',
  extras_require={
    'test': ['pytest'],
  },
  entry_points={
    'console_scripts': [
      'control_node = sr_amr_control.main:main',
    ],
  },
)
