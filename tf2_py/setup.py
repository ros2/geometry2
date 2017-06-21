#!/usr/bin/env python

from setuptools import setup

setup(
    name='tf2_py',
    version='0.5.15',
    packages=[],
    py_modules=[],
    install_requires=['setuptools', 'geometry_msgs', 'tf2_msgs'],
    maintainer='Tully Foote',
    maintainer_email='tfoote@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Python API to access tf2 buffers.',
    license='BSD'
)
