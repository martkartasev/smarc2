from setuptools import find_packages, setup
import glob, os

package_name = 'smarc_bt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ozer Ozkahraman',
    maintainer_email='ozero@kth.se',
    description='The Behaviour Tree for varius SMaRC vehicles. Usually wet.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "smarc_bt = smarc_bt.bt.ros_bt:smarc_bt",
            "test_ros_vehicle = smarc_bt.vehicles.ros_vehicle:test_ros_vehicle",
            "test_sam_auv = smarc_bt.vehicles.sam_auv:test_sam_auv",
            "test_bt_conditions = smarc_bt.bt.ros_bt:test_bt_conditions",
            "send_test_mission = smarc_bt.mission.ros_mission_updater:send_test_mission_control",
            "test_dubins_planner_caller = smarc_bt.mission.ros_dubins_planner_caller:test_dubins_planner_caller"
        ],
    },
)
