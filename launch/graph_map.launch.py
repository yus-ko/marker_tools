import launch
from launch_ros.actions import LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package = 'marker_tools'
    package_dir = get_package_share_directory(package)
    config = os.path.join(package_dir,'params','graph_map.yaml')

    return launch.LaunchDescription([
        LifecycleNode(
            package=package,
            executable='graph_map',
            name='graph_map',
            namespace='',
            output='screen',
            parameters=[
                {"frame_id_global": "map"},
                {"marker_yaml_path": config},
                {"mesh_resource_files": ['file:///opt/ros/humble/share/rviz_default_plugins/test_meshes/pr2-base.dae', 
                                         'package://rviz_default_plugins/test_meshes/pr2-base.dae']}
            ]
        )
    ])