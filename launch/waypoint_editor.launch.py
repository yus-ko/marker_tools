import launch
from launch_ros.actions import LifecycleNode
from os.path import expanduser

def generate_launch_description():
    return launch.LaunchDescription([
        LifecycleNode(
            package='marker_tools',
            executable='waypoint_editor',
            name='waypoint_editor',
            namespace='',
            output='screen',
            parameters=[
                {"frame_id_global": "map"},
                {"marker_yaml_path": expanduser("~")+"/waypoints.yaml"},
                {"mesh_resource_files": ['file:///opt/ros/humble/share/rviz_default_plugins/test_meshes/pr2-base.dae', 
                                         'package://rviz_default_plugins/test_meshes/pr2-base.dae']}
            ]
        )
    ])