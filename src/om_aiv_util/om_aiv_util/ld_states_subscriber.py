import rclpy
import sys
import time
from std_msgs.msg import String
from om_aiv_util.socket_listener import *
from om_aiv_msg.msg import Status, Location
from rclpy.node import Node

class LdStateSubscriber(Node):
    def __init__(self, listener):
        super().__init__('ld_states_subscriber_node')
        #self.listener = listener
        self.subscription = self.create_subscription(String, "ldarcl_odom",self.timer1_callback,10)
        
    def timer1_callback(self,msg):
        self.get_logger('odometer '%msg.data)
        events = self.listener.selector.select()
        #self.get_logger().info(str(events))
        for key, mask in events:
            self.listener.process_events(mask)
        self.pub_odometer()


def main(args=None):
    rclpy.init(args=args)

    ld_state_subscriber = LdStateSubscriber()

    rclpy.spin(ld_state_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ld_state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()