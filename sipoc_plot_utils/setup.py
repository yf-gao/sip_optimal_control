from setuptools import find_packages, setup

package_name = 'sipoc_plot_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yunfan',
    maintainer_email='rubygaoyunfan@gmail.com',
    description='Visualization utilities',
    license='AGPL-3.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
