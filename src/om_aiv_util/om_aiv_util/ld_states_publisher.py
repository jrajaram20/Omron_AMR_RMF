#!/usr/bin/env python
from dis import dis
from math import dist
import string


from numpy import float16, float32, float64
import rclpy
import sys
import time
from std_msgs.msg import String
from std_msgs.msg import *
from om_aiv_util.socket_listener import *
from om_aiv_msg.msg import Status, Location
from rclpy.node import Node


class LdStatePublisher(Node):
    def __init__(self, listener):
        super().__init__('ld_states_publisher_node')
        self.listener = listener
        self.status_pub = self.create_publisher(Status, "ldarcl_status", 10)
        self.vel_pub = self.create_publisher(String, "ldarcl_velocity", 10)
        self.dtg_pub = self.create_publisher(String, "ldarcl_distancetogoal", 10)
        self.ttg_pub = self.create_publisher(String, "ldarcl_timetogoal", 10)
        #self.odom_pub = self.create_publisher(String, "ldarcl_odom", 10)
        #self.app_fault_query_pub = self.create_publisher(String, "ldarcl_application_fault_query", 10)
        #self.faults_get_pub = self.create_publisher(String, "ldarcl_faults_get", 10)
        #self.query_faults_pub = self.create_publisher(String, "ldarcl_query_faults", 10)
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("LD_States_Publisher is up!")

    def timer_callback(self):
        events = self.listener.selector.select()
        #self.get_logger().info(str(events))
        for key, mask in events:
            self.listener.process_events(mask)

        self.pub_status()
        self.pub_vel()
        #self.pub_dtg()
        #self.pub_odometer()
        #self.pub_app_fault_query()
        #self.pub_faults_get()
        #self.pub_query_faults()

    
    def pub_status(self):
        status_msg = Status()
        loc_msg = Location()
        try:
            #status_ext = self.listener.get_response(b"ExtendedStatusForHumans")
            status_status = self.listener.get_response(b"Status")
            status_batt = self.listener.get_response(b"StateOfCharge")
            status_loc = self.listener.get_response(b"Location")
            status_loc_score = self.listener.get_response(b"LocalizationScore")
            status_temp = self.listener.get_response(b"Temperature")
            #status_speed = self.listener.get_response(b"TransVel")
            #status_distance  = self.listener.get_response(b"Distance")
        except KeyError:
            pass
        else:
            status_msg.status = status_status[0].decode()
    
            #status_msg.extended_status = status_ext[0].decode()
            status_msg.state_of_charge = float(status_batt[0])
            status_msg.localization_score = float(status_loc_score[0])
            status_msg.temperature = float(status_temp[0])
            #status_msg.transvel = float(status_speed[0])
            #status_msg.distance = float(status_distance[0])
            # Parse location values
            values = status_loc[0].split()
            if len(values) == 3:
                try:
                    loc_msg.x = float(values[0])
                    loc_msg.y = float(values[1])
                    loc_msg.theta = float(values[2])
                except ValueError:
                    loc_msg.x = 0
                    loc_msg.y = 0
                    loc_msg.theta = 0
                    print("Value error with location coordinates. Setting them to zeroes.")
                    pass
                else:
                    status_msg.location = loc_msg
        finally:
            self.status_pub.publish(status_msg)

    """
    
    """
    def pub_vel(self):
        time1 = String()
        speed1 = String()
        distance1 = String()
        try:        
            vel_str = self.listener.get_response(b"TransVel")
            dist_str= self.listener.get_response(b"Distance")
        except KeyError:
            pass
        else:
            speed1.data = vel_str[0]
            distance1.data = dist_str[0]
            if float(vel_str[0]) > 20.0 and float(dist_str[0]) > 300.0:
                time_str = format(float(dist_str[0])/float(vel_str[0]),".2f")
                time1.data = str(time_str)
                
            else:
                time1.data = "0.0"
            #s = distance1/speed1
            #time1 = str(s)
            #print(distance/speed)
        finally:
            self.vel_pub.publish(speed1)
            self.dtg_pub.publish(distance1)
            self.ttg_pub.publish(time1)
    # """

    # """
    def pub_dtg(self):
        try:
            dist_str= self.listener.get_response(b"Distance")
            #distance = float32(dist_str[0])
            print("dd",dist_str)
        except KeyError:
            pass
        else:
            querystring1 = String()
            querystring1 = dist_str[0]
            print("dis",querystring1)
            #querystring1.data = (' '.join([bytevalue.decode() for bytevalue in query1]))
            self.vel_pub.publish(querystring1)
    # """
            #self.get_logger().info(goalstring.data)

    # def pub_odometer(self):
    #     try:
    #         odom = self.listener.get_response(b"Odometer")
    #     except KeyError:
    #         pass
    #     else:
    #         odomstring = String()
    #         odomstring.data = (' '.join([bytevalue.decode() for bytevalue in odom]))
    #         self.odom_pub.publish(odomstring)

    # def pub_app_fault_query(self):
    #     try:
    #         query = self.listener.get_response(b"ApplicationFaultQuery")
    #     except KeyError:
    #         pass
    #     else:
    #         querystring = String()
    #         querystring.data = (' '.join([bytevalue.decode() for bytevalue in query]))
    #         self.app_fault_query_pub.publish(querystring)

    # def pub_faults_get(self):
    #     try:
    #         faults = self.listener.get_response(b"FaultList")
    #     except KeyError:
    #         pass
    #     else:
    #         faultstring = String()
    #         faultstring.data = (' '.join([bytevalue.decode() for bytevalue in faults]))
    #         self.faults_get_pub.publish(faultstring)

    # def pub_query_faults(self):
    #     try:
    #         faults = self.listener.get_response(b"RobotFaultQuery")
    #     except KeyError:
    #         pass
    #     else:
    #         faultstring = String()
    #         faultstring.data = (' '.join([bytevalue.decode() for bytevalue in faults]))
    #         self.query_faults_pub.publish(faultstring)
        
"""

"""
def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('ld_param_node')
    ip_address = node.declare_parameter("local_ip").value
    port = node.declare_parameter("local_port").value
    #node.get_logger().info('IP ADDRESS IS.... ' + str(ip_address))
    #node.get_logger().info('LOCAL PORT IS.... ' + str(port))
    listener = SocketListener(node, ip_address, port)
    listener.begin()

    ld_states_pub = LdStatePublisher(listener)
    
    try:
        rclpy.spin(ld_states_pub)
    except KeyboardInterrupt:
        listener.close()
        node.get_logger().info("Shutting down ARCL states publisher")
    

if __name__ == "__main__":
    main()
    """
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('ld_param_node')
    ip_address = node.get_parameter("local_ip")
    port = node.get_parameter("local_port")
    listener = SocketListener(str(ip_address), int(port))
    listener.begin()
    """
