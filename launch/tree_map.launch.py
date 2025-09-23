from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from os.path import expanduser, join
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package = 'marker_tools'
    package_dir = get_package_share_directory(package)
    config = join(package_dir,'params','tree_map.yaml')
    # config = expanduser("~") + "/tree_map.yaml"
    rviz = join(package_dir,'rviz','rviz.rviz')

    return LaunchDescription([
        LifecycleNode(
            package=package,
            executable='tree_map',
            name='tree_map',
            namespace='',
            output='screen',
            parameters=[
                {"frame_id_global": "map"},
                {"marker_yaml_path": config},
                {"mesh_resource_files": ['file:///opt/ros/humble/share/rviz_default_plugins/test_meshes/pr2-base.dae', 
                                         'package://rviz_default_plugins/test_meshes/pr2-base.dae']}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", rviz]
        )
    ])