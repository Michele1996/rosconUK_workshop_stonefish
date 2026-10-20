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
for folder in ['data/scenarios', 'data/robots', 'data/meshes',
               'data/opti_blueboat', 'data/bluerov2_unphy', 'data/textures', 'data/photogrammetry', 'data/autonomy', 'config']:
    if not os.path.isdir(folder):
        continue
    for root, dirs, names in os.walk(folder):
        found = [os.path.join(root, n) for n in names if os.path.isfile(os.path.join(root,n))]
        if found:
            data_files.append(('share/' + package_name + '/' + root, found))

setup(
    name=package_name,
    version='0.8.0',
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
        'bluerov_waypoint_demo = roscon_stonefish_workshop.bluerov_waypoint_demo:main',
        'bluerov_waypoint_test = roscon_stonefish_workshop.bluerov_waypoint_test:main',
        'autonomy_plan_check = roscon_stonefish_workshop.autonomy_plan:main',
        'bluerov_thruster_demo = roscon_stonefish_workshop.bluerov_thruster_demo:main',
        'photogrammetry_evaluate = roscon_stonefish_workshop.survey_evaluation:main',
        'photogrammetry_align = roscon_stonefish_workshop.photogrammetry_align:main',
        'photogrammetry_geometry_eval = roscon_stonefish_workshop.photogrammetry_geometry_eval:main',
        'inspection_mission = roscon_stonefish_workshop.inspection_mission:main',
        'inspection_reference = roscon_stonefish_workshop.inspection_reference:main',
        'inspection_evaluate = roscon_stonefish_workshop.inspection_evaluation:main',
        'photogrammetry_capture = roscon_stonefish_workshop.photogrammetry_capture:main',
        'survey_planner = roscon_stonefish_workshop.survey_planner:main',
        'thruster_adapter = roscon_stonefish_workshop.thruster_adapter:main',
    ]},
)
