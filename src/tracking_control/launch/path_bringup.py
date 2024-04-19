from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            executable='tracking_node',
            name='tracking_node',
            output='screen'
        )
    ])