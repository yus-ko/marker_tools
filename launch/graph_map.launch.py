import launch
from launch_ros.actions import LifecycleNode
from os.path import expanduser

def generate_launch_description():
    return launch.LaunchDescription([
        LifecycleNode(
            package='marker_tools',
            executable='graph_map',
            name='graph_map',
            namespace='',
            output='screen',
            parameters=[
                {"marker_yaml_path": expanduser("~")+"/potbot_ws/src/marker_tools/params/graph_map.yaml"}
            ]
        )
    ])