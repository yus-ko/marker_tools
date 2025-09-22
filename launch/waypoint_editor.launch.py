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
                {"marker_yaml_path": expanduser("~")+"/potbot_ws/src/marker_tools/params/waypoints.yaml"}
            ]
        )
    ])