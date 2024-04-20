import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np



class Nav2TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('nav2_trajectory_planner')
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_control_cmd = self.create_publisher(Twist, '/track_cmd_vel', 10)
        
        self.map_data = None
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        #self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.initial_pose_publisher = self.create_publisher(PoseStamped, '/initialpose', 10)
        
        # Set the initial pose of the robot
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.position.x = 0.0  # Set the initial x position
        initial_pose.pose.position.y = 0.0  # Set the initial y position
        initial_pose.pose.orientation = self.quaternion_from_euler(0.0, 0.0, 0.0)  # Set the initial orientation (roll, pitch, yaw)
        self.initial_pose_publisher.publish(initial_pose)

    def map_callback(self, msg):
        self.map_data = msg

    def goal_pose_callback(self,msg):
    # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Extract orientation
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)

        self.send_goal(x,y,yaw)

    def send_goal(self, x, y, theta):
        if self.map_data is None:
            self.get_logger().error('Map data not received yet')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = self.orientation_around_z_axis(theta)

        print("waiting for server")
        self.nav2_client.wait_for_server()
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    def orientation_around_z_axis(self, theta):
        quaternion = Quaternion()
        quaternion.w = np.cos(theta / 2.0)
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(theta / 2.0)
        return quaternion
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result')
        get_result_future = goal_handle.get_result_async()
        print("navigation complete")

    

def main(args=None):
    print("Node started")
    rclpy.init(args=args)
    node = Nav2TrajectoryPlanner()

    # Wait for the map data to be received
    while node.map_data is None:
        print("No map recived")
        rclpy.spin_once(node)

    # Set the goal pose (x, y, theta) based on the map data
    goal_x = 1.0
    goal_y = 2.0
    goal_theta = 0.0
    print("Map data recived")

    # Send the goal to Nav2
    node.send_goal(goal_x, goal_y, goal_theta)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()