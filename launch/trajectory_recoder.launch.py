import launch
from launch_ros.actions import LifecycleNode
from os.path import expanduser

def generate_launch_description():
    return launch.LaunchDescription([
        LifecycleNode(
            package='marker_tools',
            executable='trajectory_recoder',
            name='recoder',
            namespace='',
            output='screen',
            parameters=[
                {"marker_yaml_path": expanduser("~")+"/potbot_ws/src/marker_tools/params/interactive_markers.yaml"}
            ]
        )
    ])