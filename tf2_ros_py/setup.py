from setuptools import find_packages
from setuptools import setup

package_name = 'tf2_ros_py'

setup(
    name=package_name,
    version='0.39.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Eitan Marder-Eppstein',
    author_email='',
    maintainer='Alejandro Hernandez Cordero, Chris Lalancette',
    maintainer_email='alejandro@openrobotics.org, clalancette@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'This package contains the ROS Python bindings for the tf2 library.'
    ),
    license='BSD',
    tests_require=['pytest'],
)
