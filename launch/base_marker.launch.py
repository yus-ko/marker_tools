import launch
from launch_ros.actions import LifecycleNode
from os.path import expanduser

def generate_launch_description():
    return launch.LaunchDescription([
        LifecycleNode(
            package='marker_tools',
            executable='base_marker',
            name='base_marker',
            namespace='',
            output='screen',
            parameters=[
                {"frame_id_global": "map"},
                {"marker_yaml_path": expanduser("~")+"/interactive_markers.yaml"},
                {"mesh_resource_files": ['file:///opt/ros/humble/share/rviz_default_plugins/test_meshes/pr2-base.dae', 
                                         'package://rviz_default_plugins/test_meshes/pr2-base.dae']}
            ]
        ),
        # launch_ros.actions.Node(
        #     package='marker_tools',
        #     executable='base_marker',
        #     name='base_marker',
        #     namespace='imns',
        #     output='screen'
        # )
    ])