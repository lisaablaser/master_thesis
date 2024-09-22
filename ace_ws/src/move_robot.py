import rclpy
from rclpy.node import Node
from automatic_cell_explorer.srv import MoveToNbv  # Import the service type
from geometry_msgs.msg import PoseStamped  # Import the PoseStamped message type
from std_msgs.msg import Header  # Import Header for the PoseStamped


class MoveRobotClient(Node):

    def __init__(self):
        super().__init__("move_robot_client")
        # Create a client for the MoveToNbv service
        self.client = self.create_client(MoveToNbv, "/move_robot_to_pose")

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

    def send_request(self, pose_stamped):
        # Create a request object
        request = MoveToNbv.Request()
        request.pose = pose_stamped  # Set the PoseStamped message in the request

        # Send the request asynchronously
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            return self.future.result()
        else:
            self.get_logger().error("Service call failed")


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the client
    move_robot_client = MoveRobotClient()

    # Define the PoseStamped message
    pose_stamped = PoseStamped()

    # Fill in the header (optional frame ID and timestamp)
    pose_stamped.header = Header()
    pose_stamped.header.frame_id = (
        "world"  # Coordinate frame, e.g., 'map' or 'base_link'
    )
    pose_stamped.header.stamp = (
        rclpy.clock.Clock().now().to_msg()
    )  # Set the current timestamp
    print("time: ", pose_stamped.header.stamp)
    # Set the pose (position and orientation)
    pose_stamped.pose.position.x = 0.4  # Replace with desired X coordinate
    pose_stamped.pose.position.y = 0.4  # Replace with desired Y coordinate
    pose_stamped.pose.position.z = 1.5  # Replace with desired Z coordinate

    # Set the orientation using a quaternion
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0  # Identity quaternion for no rotation

    # Send the request and receive the result
    result = move_robot_client.send_request(pose_stamped)

    if result:
        move_robot_client.get_logger().info("Result: %s" % str(result))

    move_robot_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
