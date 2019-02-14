#!/usr/bin/env python

import os

from setuptools import find_packages, setup

package_name = 'py_trees_ros_tutorials'

install_requires = [] if os.environ.get('AMENT_PREFIX_PATH') else [
    # build
    'setuptools',
    # runtime
]

setup(
    name=package_name,
    version='0.1.0',  # also update package.xml and version.py
    packages=find_packages(exclude=['tests*', 'docs*', 'launch*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # launchers
        ('share/' + package_name + '/launch',
         [
             'launch/mock_robot.launch.py',
         ]
         ),
        # global scripts
        #   note: package specific scripts use the entry_points
        #   configured by setup.cfg
        # ('bin',
        #  [
        #     'scripts/py-trees-blackboard-watcher',
        #     'scripts/py-trees-tree-watcher',
        #     'scripts/py-trees-latched-echo'
        #  ]
        #  ),
    ],
    package_data={'py_trees_ros_tutorials': ['mock/gui/*']},
    install_requires=install_requires,
    extras_require={},
    author='Daniel Stonier',
    maintainer='Daniel Stonier <d.stonier@gmail.com>',
    url='https://github.com/splintered-reality/py_trees_ros_tutorials',
    keywords=['ROS', 'ROS2' 'behaviour-trees'],
    zip_safe=True,
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries'
    ],
    description=(
        "Tutorials for py_trees on ROS2."
    ),
    long_description=(
        "Tutorials demonstrating usage of py_trees in ROS and more generally,"
        "behaviour trees for robotics."
    ),
    license='BSD',
    # test_suite = 'nose.collector',
    # tests_require=['nose', 'pytest', 'flake8', 'yanc', 'nose-htmloutput']
    # tests_require=['pytest'],
    entry_points={
         'console_scripts': [
             # These are redirected to lib/<package_name> by setup.cfg
             'mock-battery = py_trees_ros.mock.battery:main',
             'mock-dashboard = py_trees_ros.mock.dashboard:main',
             'mock-led-strip = py_trees_ros.mock.led_strip:main',
         ],
     },
)
