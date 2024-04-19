from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/maps/yahboomcar1.yaml',
        description='Full path to map yaml file to load'
    )
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                {'expected_planner_frequency': 20.0},
                {'use_sim_time': False},
                {'planner_plugin': 'nav2_navfn_planner/NavfnPlanner'}
            ]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                {'controller_plugin': 'nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController'},
                {'min_x_velocity_threshold': 0.001},
                {'min_y_velocity_threshold': 0.5},
                {'min_theta_velocity_threshold': 0.001}
            ]
        ),
        Node(
            package='tracking_control',
            executable='Tracjectory_node',
            name='Tracjectory_node',
            output='screen'
        )
    ])