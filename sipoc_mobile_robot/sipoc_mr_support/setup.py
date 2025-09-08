import glob
from setuptools import find_packages, setup

package_name = 'sipoc_mr_support'
maps_files = glob.glob('sipoc_mr_support/maps/*.csv')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps', maps_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yunfan',
    maintainer_email='rubygaoyunfan@gmail.com',
    description='Map generation for numerical evaluation and definition of robot dynamics',
    license='AGPL-3.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
