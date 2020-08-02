#!/usr/bin/env python3

import os

from distutils import log
from setuptools import find_packages, setup
from setuptools.command.develop import develop
from setuptools.command.install import install

package_name = 'py_trees_ros_tutorials'


# This is somewhat dodgy as it will escape any override from, e.g. the command
# line or a setup.cfg configuration. It does however, get us around the problem
# of setup.cfg influencing requirements install on rtd installs
#
# TODO: should be a way of detecting whether scripts_dir has been influenced
# from outside
def redirect_install_dir(command_subclass):

    original_run = command_subclass.run

    def modified_run(self):
        try:
            old_script_dir = self.script_dir  # develop
        except AttributeError:
            old_script_dir = self.install_scripts  # install
        # TODO: A more intelligent way of stitching this together...
        # Warning: script_dir is typically a 'bin' path alongside the
        # lib path, if ever that is somewhere wildly different, this
        # will break.
        # Note: Consider making use of self.prefix, but in some cases
        # that is mislading, e.g. points to /usr when actually
        # everything goes to /usr/local
        new_script_dir = os.path.abspath(
            os.path.join(
                old_script_dir, os.pardir, 'lib', package_name
            )
        )
        log.info("redirecting scripts")
        log.info("  from: {}".format(old_script_dir))
        log.info("    to: {}".format(new_script_dir))
        if hasattr(self, "script_dir"):
            self.script_dir = new_script_dir  # develop
        else:
            self.install_scripts = new_script_dir  # install
        original_run(self)

    command_subclass.run = modified_run
    return command_subclass


@redirect_install_dir
class OverrideDevelop(develop):
    pass


@redirect_install_dir
class OverrideInstall(install):
    pass


def gather_launch_files():
    data_files = []
    for root, unused_subdirs, files in os.walk('launch'):
        destination = os.path.join('share', package_name, root)
        launch_files = []
        for file in files:
            pathname = os.path.join(root, file)
            launch_files.append(pathname)
        data_files.append((destination, launch_files))
    return data_files


setup(
    cmdclass={
        'develop': OverrideDevelop,
        'install': OverrideInstall
    },
    name=package_name,
    # also update package.xml (version and website url), version.py and conf.py
    version='2.1.0',
    packages=find_packages(exclude=['tests*', 'docs*', 'launch*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', [
            'resources/py_trees_ros_tutorials']),
    ] + gather_launch_files(),
    package_data={'py_trees_ros_tutorials': ['mock/gui/*']},
    install_requires=[],  # it's all lies (c.f. package.xml, but no use case for this yet)
    extras_require={},
    author='Daniel Stonier',
    maintainer='Daniel Stonier <d.stonier@gmail.com>',
    url='https://github.com/splintered-reality/py_trees_ros_tutorials',
    keywords=['ROS', 'ROS2', 'behaviour-trees'],
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
            # Tutorial Nodes
            'tree-data-gathering = py_trees_ros_tutorials.one_data_gathering:tutorial_main',
            'tree-battery-check = py_trees_ros_tutorials.two_battery_check:tutorial_main',
            'tree-action-clients = py_trees_ros_tutorials.five_action_clients:tutorial_main',
            'tree-context-switching = py_trees_ros_tutorials.six_context_switching:tutorial_main',
            'tree-docking-cancelling-failing = py_trees_ros_tutorials.seven_docking_cancelling_failing:tutorial_main',
            'tree-dynamic-application-loading = py_trees_ros_tutorials.eight_dynamic_application_loading:tutorial_main',
        ],
    },
)
