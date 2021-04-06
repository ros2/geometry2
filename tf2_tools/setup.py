from setuptools import setup

package_name = 'tf2_tools'

setup(
    name=package_name,
    version='0.17.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Wim Meeussen',
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='tf2_tools for debugging',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'view_frames = tf2_tools.view_frames:main',
        ],
    },
)
