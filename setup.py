#!/usr/bin/env python

from setuptools import find_packages, setup

package_name = 'py_trees_ros_tutorials'

install_requires = [
    # build
    'setuptools',
    # runtime
    #  - can only add pure python package dependencies here
    #  - runtime scripts will fail if not found
    #      (so some utility exists to duplicate package.xml <exec_depend> here)
    #  - doesn't cover ros2 python packages installed by ament_cmake
    #    - though presumably this could be fixed
    'launch',
    'launch_ros',
    'py_trees',
    # 'py_trees_ros_interfaces',
    # 'rclpy',
    'ros2launch',
    'ros2param',
    'ros2run',
    'ros2service',
    'ros2topic',
    # 'std_msgs'
]

setup(
    name=package_name,
    version='0.1.0',  # also update package.xml and version.py
    packages=find_packages(exclude=['tests*', 'docs*', 'launch*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # launchers
        # ('share/' + package_name + '/launch',
        #  [
        #      'launch/mock_robot.launch.py',
        #  ]
        #  ),
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
    # test_suite="tests"
    # tests_require=['nose', 'pytest', 'flake8', 'yanc', 'nose-htmloutput']
    entry_points={
         'console_scripts': [
             # These are redirected to lib/<package_name> by setup.cfg
             # Mocks
             'mock-battery = py_trees_ros_tutorials.mock.battery:main',
             'mock-dashboard = py_trees_ros_tutorials.mock.dashboard:main',
             'mock-docking-controller = py_trees_ros_tutorials.mock.dock:main',
             'mock-led-strip = py_trees_ros_tutorials.mock.led_strip:main',
             'mock-move-base = py_trees_ros_tutorials.mock.move_base:main',
             'mock-rotation-controller = py_trees_ros_tutorials.mock.rotate:main',
             'mock-safety-sensors = py_trees_ros_tutorials.mock.safety_sensors:main',
             # Mock Tests
             'mock-dock-client = py_trees_ros_tutorials.mock.actions:dock_client',
             'mock-move-base-client = py_trees_ros_tutorials.mock.actions:move_base_client',
             'mock-rotate-client = py_trees_ros_tutorials.mock.actions:rotate_client',
             # Mock Launcher
             'launch-mock-robot = py_trees_ros_tutorials.mock.launch:main',
             # Tutorial Nodes
             'tree-data-gathering = py_trees_ros_tutorials.one_data_gathering:tutorial_main',
             # Tutorial Launchers (directly runnable)
             'tutorial-one-data-gathering = py_trees_ros_tutorials.one_data_gathering:launch_main',
             # Other
             'testies = py_trees_ros_tutorials.testies:main',
             'launch-testies = py_trees_ros_tutorials.launch_testies:main',
         ],
     },
)
