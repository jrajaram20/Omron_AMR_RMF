import rclpy
import sys
import time
from std_msgs.msg import String
#from om_aiv_util.socket_listener import *
#from om_aiv_msg.msg import Status, Location
from rclpy.node import Node

class LdStateSubscriber(Node):
    def __init__(self):
        super().__init__('ldvel')
        #self.listener = listener
        self.subscription = self.create_subscription(String, "ldarcl_timetogoal",self.timer1_callback,10)
        
    def timer1_callback(self,msg):
        #print(msg.data)
        #msg1 = str(msg.data)
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.get_logger().info('robotspeed "%s"' %msg.data)     

def main(args=None):
    rclpy.init(args=args)

    ldvel = LdStateSubscriber()

    rclpy.spin(ldvel)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ldvel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
