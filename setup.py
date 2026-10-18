from setuptools import setup
from glob import glob
import os

package_name = 'roscon_stonefish_workshop'

def files(pattern):
    return [p for p in glob(pattern, recursive=True) if os.path.isfile(p)]

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', files('launch/*.launch.py')),
]
for folder in ['data/scenarios', 'data/robots', 'data/meshes', 'data/opti_blueboat', 'data/textures', 'config']:
    fs = files(folder + '/**/*')
    if fs:
        data_files.append(('share/' + package_name + '/' + folder, fs))

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Michele Grimaldi',
    maintainer_email='michele.grimaldi@hw.ac.uk',
    description='ROSCon UK 2026 hands-on workshop with ROS 2 and Stonefish.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'blueboat_pid = roscon_stonefish_workshop.blueboat_pid:main',
        'blueboat_thruster_intro = roscon_stonefish_workshop.blueboat_thruster_intro:main',
        'blueboat_thruster_demo = roscon_stonefish_workshop.blueboat_thruster_intro:main',
        'race_start = roscon_stonefish_workshop.race_start:main',
        'race_export = roscon_stonefish_workshop.race_export:main',
        'blueboat_aruco_detector = roscon_stonefish_workshop.blueboat_aruco_detector:main',
        'blueboat_race_monitor = roscon_stonefish_workshop.blueboat_race_monitor:main',
        'blueboat_thruster_adapter = roscon_stonefish_workshop.blueboat_thruster_adapter:main',
        'bluerov_pid = roscon_stonefish_workshop.bluerov_pid:main',
        'survey_planner = roscon_stonefish_workshop.survey_planner:main',
        'thruster_adapter = roscon_stonefish_workshop.thruster_adapter:main',
    ]},
)
