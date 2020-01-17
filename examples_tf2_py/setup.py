from setuptools import setup

package_name = 'examples_tf2_py'

setup(
    name=package_name,
    version='0.12.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/broadcasters.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Shane Loretz',
    author_email='sloretz@openrobotics.org',
    maintainer='Tully Foote',
    maintainer_email='tfoote@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Has examples of using the tf2 python api.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_broadcaster = examples_tf2_py.static_broadcaster:main',
            'dynamic_broadcaster = examples_tf2_py.dynamic_broadcaster:main',
            'frame_dumper = examples_tf2_py.frame_dumper:main',
            'waits_for_transform = examples_tf2_py.waits_for_transform:main',
            'blocking_waits_for_transform = examples_tf2_py.blocking_waits_for_transform:main',
            'async_waits_for_transform = examples_tf2_py.async_waits_for_transform:main',
        ],
    },
)
