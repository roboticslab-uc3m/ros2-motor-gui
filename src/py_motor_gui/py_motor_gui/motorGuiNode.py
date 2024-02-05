import rclpy
import math
import itertools

from rclpy.node import Node

# Import Position message type from yarp_control_msgs, used for publishing position data
from yarp_control_msgs.msg import Position

class WindowSubscriber(Node):
    """
    A ROS node for subscribing to joint positions and publishing them to specified topics.
    This class inherits from Node and is responsible for handling the communication with ROS.
    """

    def __init__(self):
        """
        Constructor for the WindowSubscriber class. Initializes the ROS node and publishers.
        """

        super().__init__('window_subscriber')

        # Create publishers for different robot parts. Each publisher will send messages to a specific topic
        # related to a part of the robot, such as head, arms, trunk, and legs. The queue size is set to 10.
        self.head_publisher_     = self.create_publisher(Position, '/teoSim/head/position', 10)
        self.rightArm_publisher_ = self.create_publisher(Position, '/teoSim/rightArm/position', 10)
        self.leftArm_publisher_  = self.create_publisher(Position, '/teoSim/leftArm/position', 10)
        self.trunk_publisher_    = self.create_publisher(Position, '/teoSim/trunk/position', 10)
        self.rightLeg_publisher_ = self.create_publisher(Position, '/teoSim/rightLeg/position', 10)
        self.leftLeg_publisher_  = self.create_publisher(Position, '/teoSim/leftLeg/position', 10)

    def publishPosition(self, articulation, position, speed, extremity):
        """
        Publishes the position and speed of a given articulation to the appropriate ROS topic.

        :param articulation: The name of the articulation/joint.
        :param position: The target position for the articulation (in degrees).
        :param speed: The reference speed for moving to the target position.
        :param extremity: The part of the robot's body the articulation belongs to (e.g., neck, rightArm).
        """

        # Create a new Position message
        msg = Position()

        # Check if position and speed are provided
        if (position != '' and speed != ''):
            # Set the names, positions (converted to radians), and reference velocities for the message
            msg.names = [str(articulation)]
            msg.positions = [float(position) * (math.pi/180)]
            msg.ref_velocities = [float(speed)/100]

            # Publish the message to the appropriate topic based on the extremity parameter
            if (extremity == 'neck'):
                self.head_publisher_.publish(msg)
            elif (extremity == 'rightArm'):
                self.rightArm_publisher_.publish(msg)
            elif (extremity == 'leftArm'):
                self.leftArm_publisher_.publish(msg)
            elif (extremity == 'trunk'):
                self.trunk_publisher_.publish(msg)
            elif (extremity == 'rightLeg'):
                self.rightLeg_publisher_.publish(msg)
            elif (extremity == 'leftLeg'):
                self.leftLeg_publisher_.publish(msg)

def main(args=None):
    """
    Main function to initialize ROS, create the WindowSubscriber node, and spin it to keep it alive.
    """

    rclpy.init(args=args)
    window_subscriber = WindowSubscriber()
    rclpy.spin(window_subscriber)

    window_subscriber.destroy_node()
    rclpy.shutdown()

if(__name__ == '__main__'):
    main()