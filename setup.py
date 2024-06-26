from setuptools import setup

package_name = 'thesis_licence'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/world_new_obstcl.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/khepera.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebi',
    maintainer_email='seby.stoica28@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = thesis_licence.my_robot_driver:main',
            'tf2_transform = thesis_licence.tf2_transform:main',
            'obstacle = thesis_licence.obstacle:main'

        ],
    },
)
