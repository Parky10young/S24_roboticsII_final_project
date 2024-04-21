import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped


class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'waypoint_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

        self.current_pos_subscriber = self.create_subscription(String, '/nav2_status', self.current_position_callback, 10)
        self.object_pose_subscriber = self.create_subscription(PoseStamped, '/detected_color_object_pose', self.object_pose_callback, 10)

    def timer_callback(self):
        #loop between waypoint 1 and start position
        #go to waypoint 1
        if self.i==1:
            print("yes")
            waypoint = Float32MultiArray()
            waypoint.data = [3.0,3.0,np.pi/2]
            self.publisher_.publish(waypoint)
            self.get_logger().info('Publishing: "%s"' % waypoint.data)
            self.i == 1
            """
            if  curr_pos== "Arrived":
                self.i = 0
                return
            return
            """
        #go to start
        elif self.i==0:
            waypoint = Float32MultiArray()
            waypoint.data = [0.0,0.0,0.0]
            self.publisher_.publish(waypoint)
            self.get_logger().info('Publishing: "%s"' % waypoint.data)
            self.i = 0
            """
            if  curr_pos== "Arrived":
                self.i = 1
                return
            return
            """

        
    def current_position_callback(self, msg):
        #check if robot has arrived to waypoint
        curr_pos = msg.data

    def object_pose_callback(self, detected_obj_pose):
        #print object coordinates 
        '''
         detected_obj_pose.pose.position.x = cp_robot[0]
            detected_obj_pose.pose.position.y = cp_robot[1]
            detected_obj_pose.pose.position.z = cp_robot[2]
            '''
        x = detected_obj_pose.pose.position.x
        y = detected_obj_pose.pose.position.y
        z = detected_obj_pose.pose.position.z
        print("Object Position")
        print("X:",x,"y:",y,"z:",z)

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    rclpy.spin(waypoint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
