import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

class PolicyPublisher(Node):

    def __init__(self, policy):
        super().__init__('policy_publisher')
        self.policy = policy
        self.ee_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # We publish end-effector commands to this topic
        self.publisher_ = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)

        # This will get the next action from the policy every 0.1 seconds
        self.policy_timer = self.create_timer(0.5, self.policy_callback)

        # This will check the robot's current end-effector pose every 0.1 seconds
        self.tf_timer = self.create_timer(0.1, self.eef_callback)

    def eef_callback(self):
        # Look up the end-effector pose using the transform tree
        try:
            t = self.tf_buffer.lookup_transform(
                "link_base", #to_frame_rel,
                "link_eef", #from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not get transform: {ex}')
            rclpy.time.sleep(1)
            return

        self.ee_pose = t.transform.translation


    def policy_callback(self):
        if self.ee_pose is None:
            print("Waiting for ee pose...")
            return

        action = self.policy(self.ee_pose)

        # Convert the action vector into a Twist message
        twist = TwistStamped()
        twist.twist.linear.x = action[0]
        twist.twist.linear.y = action[1]
        twist.twist.linear.z = action[2]
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        twist.header.frame_id = "link_base"
        twist.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(twist)

def main(args=None):
    # Sample policy that will move the end-effector in a box-like shape
    def policy(state):
        if state.x < 0.25 and state.y < 0.25:
            action = [0.0, 0.1, 0.0]
        elif state.y >= 0.25 and state.x < 0.6:
            action = [0.1, 0.0, 0.0]
        elif state.x >= 0.6 and state.y > -0.25:
            action = [0.0, -0.1, 0.0]
        else:
            action = [-0.1, 0.0, 0.0]
        return action

    rclpy.init(args=args)

    policy_publisher = PolicyPublisher(policy)

    rclpy.spin(policy_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    policy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
