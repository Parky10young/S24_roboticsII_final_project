import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_subscriber = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_markers', 10)
        self.map_data = None
        self.path_data = None
        self.goal_data = None

    def map_callback(self, msg):
        self.map_data = msg
        self.publish_markers()

    def path_callback(self, msg):
        self.path_data = msg
        self.publish_markers()

    def goal_callback(self, msg):
        self.goal_data = msg
        self.publish_markers()

    def publish_markers(self):
        if self.map_data is None or self.path_data is None or self.goal_data is None:
            return

        marker_array = MarkerArray()

        # Create map marker
        map_marker = Marker()
        map_marker.header.frame_id = self.map_data.header.frame_id
        map_marker.type = Marker.CUBE
        map_marker.action = Marker.ADD
        map_marker.scale.x = self.map_data.info.resolution * self.map_data.info.width
        map_marker.scale.y = self.map_data.info.resolution * self.map_data.info.height
        map_marker.scale.z = 0.01
        map_marker.color.r = 0.5
        map_marker.color.g = 0.5
        map_marker.color.b = 0.5
        map_marker.color.a = 1.0
        map_marker.pose.position.x = self.map_data.info.origin.position.x + map_marker.scale.x / 2
        map_marker.pose.position.y = self.map_data.info.origin.position.y + map_marker.scale.y / 2
        map_marker.pose.position.z = 0.0
        map_marker.pose.orientation.w = 1.0
        marker_array.markers.append(map_marker)

        # Create path marker
        path_marker = Marker()
        path_marker.header.frame_id = self.path_data.header.frame_id
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1
        path_marker.color.r = 1.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        path_marker.points = [pose.pose.position for pose in self.path_data.poses]
        marker_array.markers.append(path_marker)

        # Create goal marker
        goal_marker = Marker()
        goal_marker.header.frame_id = self.goal_data.header.frame_id
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.scale.x = 0.5
        goal_marker.scale.y = 0.5
        goal_marker.scale.z = 0.5
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        goal_marker.pose = self.goal_data.pose
        marker_array.markers.append(goal_marker)

        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()