import launch
import launch.actions
import launch_ros.actions
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return launch.LaunchDescription([
        # launch_ros.actions.LifecycleNode(
        #     package='test_pkg',
        #     executable='interactive_marker',
        #     name='interactive_marker',
        #     namespace='',
        #     output='screen'
        # ),
        launch_ros.actions.Node(
            package='marker_tools',
            executable='interactive_marker',
            name='interactive_marker',
            namespace='imns',
            output='screen'
        )
    ])