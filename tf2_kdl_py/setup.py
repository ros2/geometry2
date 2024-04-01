from setuptools import find_packages, setup

package_name = 'tf2_kdl_py'

setup(
    name=package_name,
    version='0.36.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Hernandez Cordero, Chris Lalancette',
    maintainer_email='alejandro@openrobotics.org, clalancette@openrobotics.org',
    description='PyKDL binding for tf2',
    license='Apache-2.0',
    tests_require=['pytest'],
)
