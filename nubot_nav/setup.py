from setuptools import find_packages, setup

package_name = 'nubot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manual_explore.launch.xml']),
        ('share/' + package_name + '/config', ['launch/explore.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/config', ['config/manual_explore.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Khachatryan',
    maintainer_email='davidkh@u.northwestern.edu',
    description='Exploring the navigation stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explore = nubot_nav.explore:explore_entry',
        ],
    },
)
