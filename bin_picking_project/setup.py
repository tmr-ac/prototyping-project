from setuptools import setup
import glob
import os

package_name = 'bin_picking_project'
submodules = "bin_picking_project/image_processing"

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf_model/meshes'), glob.glob('urdf_model/meshes/*.dae')),
        (os.path.join('share', package_name, 'urdf_model'), glob.glob('urdf_model/robot_model.urdf')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/config.rviz')),
        (os.path.join('share', package_name, 'environment'), glob.glob('environment/*.dae')),
        (os.path.join('share', package_name, 'table_images'), glob.glob('table_images/*.jpg')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Orhun Oezcan, Timur Kuzu, Dominik Leitner',
    maintainer_email='orhun.oezcan@rwth-aachen.de, timur.kuzu@rwth-aachen.de, dominik.leitner@rwth-aachen.de',
    description='This package contains our bin-picking Prototyping Project of winter semester 2022 / 2023.',
    license='RWTH Aachen University',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':[
        "object_detection = bin_picking_project.object_detection:main",
        "inverse_kinematics = bin_picking_project.inverse_kinematics:main",
        "environment_marker = bin_picking_project.environment_marker:main",
        "object_marker = bin_picking_project.object_marker:main",
        "image_publisher = bin_picking_project.image_publisher:main"
        ],
    },
)
