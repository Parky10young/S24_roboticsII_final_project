import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler

class Nav2TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('nav2_trajectory_planner')
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.orientation_around_z_axis(theta)

        self.nav2_client.wait_for_server()
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def orientation_around_z_axis(self, theta):
        return quaternion_from_euler(0, 0, theta)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.status == NavigateToPose.Result.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
        elif result.status == NavigateToPose.Result.STATUS_ABORTED:
            self.get_logger().error('Goal was aborted')
        elif result.status == NavigateToPose.Result.STATUS_CANCELED:
            self.get_logger().error('Goal was canceled')
        else:
            self.get_logger().error('Unknown result status')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2TrajectoryPlanner()

    # Set the goal pose (x, y, theta)
    goal_x = 1.0
    goal_y = 1.0
    goal_theta = 0.0

    # Send the goal to Nav2
    node.send_goal(goal_x, goal_y, goal_theta)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()