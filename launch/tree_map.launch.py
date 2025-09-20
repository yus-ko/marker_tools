import launch
from launch_ros.actions import LifecycleNode
from os.path import expanduser

def generate_launch_description():
    return launch.LaunchDescription([
        LifecycleNode(
            package='marker_tools',
            executable='tree_map',
            name='tree_map',
            namespace='',
            output='screen',
            parameters=[
                {"marker_yaml_path": expanduser("~")+"/potbot_ws/src/marker_tools/params/tree_map.yaml"}
            ]
        )
    ])