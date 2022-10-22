#!/usr/bin/env python
from dis import dis
from math import dist
import stat
import string


from numpy import float16, float32, float64
import rclpy
import sys
import time
from std_msgs.msg import String
from std_msgs.msg import *
from om_state_pub.ld_arcl import *
from om_aiv_msg.msg import Location, Status#, Locationrmf,  Robotstate
from rclpy.node import Node


class LdStatePublisher(Node):
    def __init__(self, listener,_robotname):
        super().__init__('ld_param_node')
        self.listener = listener
        self.robotname = _robotname
        self.levelname = "L1"
        self.mapname = "loglab"
        self.timetogoal = 0
        self.status_pub = self.create_publisher(Status, "Rstate", 10)
        #self.status_pub = self.create_publisher(Robotstate, "Rstate", 10)
        #self.vel_pub = self.create_publisher(String, self.robot_name+"_velocity", 10)
        #self.dtg_pub = self.create_publisher(String, self.robot_name+"_distancetogoal", 10)
        #self.ttg_pub = self.create_publisher(String, "timetogoal", 10)
        #self.batt_pub = self.create_publisher(String, self.listener.robot_name+"_battery", 10)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(self.robotname +" LD_Publisher is up!")

    def timer_callback(self):
        self.pub_status()
        #self.pub_vel()

    
    def pub_status(self):
        status_msg = Status()
        loc_msg = Location()
        #status_msg = Robotstate()
        #loc_msg = Locationrmf()
        self.timetogoal = self._timetogoal()
        try:
            status_name = self.robotname
            status_status = self.listener.get_response("ModeStatus")
            status_batt = self.listener.get_response("BatteryStateOfCharge")
            status_locx = self.listener.get_response("RobotX")
            status_locy = self.listener.get_response("RobotY")
            status_locth = self.listener.get_response("RobotTh")
            status_loclevname = self.levelname
            status_locmapname = self.mapname
            status_timetogoal = self.timetogoal 
            #status_speed = self.listener.get_response("TransVel")
            #status_distance  = self.listener.get_response(b"Distance")
        except KeyError:
            pass
        else:
            try:
                status_msg.name = status_name
                status_msg.status = status_status[0]
                status_msg.state_of_charge = float(status_batt[0])
                status_msg.time_to_goal = float(self.timetogoal)
                #status_msg.transvel = float(status_speed[0])
                #status_msg.distance = float(status_distance[0])
                # Parse location values
                loc_msg.x = float(status_locx[0])
                loc_msg.y = float(status_locy[0])
                loc_msg.theta = float(status_locth[0])
                loc_msg.level_name = self.levelname
                loc_msg.map_name = self.mapname
            except ValueError:
                status_msg.name = ""
                status_msg.status = "NA"
                status_msg.state_of_charge = -100.0
                status_msg.time_to_goal = 0.0
                loc_msg.x = 0
                loc_msg.y = 0
                loc_msg.theta = 0
                loc_msg.level_name = self.levelname
                loc_msg.map_name = self.mapname
                print("Value error with location coordinates. Setting them to zeroes.")
                pass
            else:
                status_msg.location = loc_msg
        finally:
            self.status_pub.publish(status_msg)
    """
    
    """
    def _timetogoal(self):
        time_str = 0.0
        try:        
            vel_str = self.listener.get_response("TransVel")
            dist_str= self.listener.get_response("Distance_to_goal")
        except KeyError:
            pass
        else:
            if float(vel_str[0]) > 20.0 and float(dist_str[0]) > 300.0:
                time_str = format(float(dist_str[0])/float(vel_str[0]),".2f")
                return time_str
            else:
                return time_str
        finally:
            return time_str

    def pub_vel(self):
        time1 = String()
        speed1 = String()
        distance1 = String()
        #loc_msg = Location()
        try:        
            vel_str = self.listener.get_response("TransVel")
            print(vel_str)
            dist_str= self.listener.get_response("Distance_to_goal")
        except KeyError:
            pass
        else:
            
            speed1.data = vel_str[0]
            distance1.data = dist_str[0]
            print(speed1.data)
            if float(vel_str[0]) > 20.0 and float(dist_str[0]) > 300.0:
                time_str = format(float(dist_str[0])/float(vel_str[0]),".2f")
                time1.data = str(time_str)
                
            else:
                time1.data = "0.0"
        finally:
            # self.dtg_pub.publish(distance1)
            # self.ttg_pub.publish(time1)
            return time1.data
    # """

    # """

def main(args = None):
    rclpy.init(args=args)
    node = rclpy.create_node('ld_param_node')
    node.get_logger().info("socket connection started")
    ip_address = node.declare_parameter("ip_address").value
    port = node.declare_parameter("port").value
    arcl_password = node.declare_parameter("arcl_passwd").value
    amr_name = node.declare_parameter("amr_name").value
    #print("@@@@@ ",amr_name)
    #amr_name = "amr1"
    node.get_logger().info('IP ADDRESS IS.... ' + str(ip_address))
    node.get_logger().info('LOCAL PORT IS.... ' + str(port))
    #arcl_password = "omron"
    listener = ConnectSocket(node, ip_address, port,arcl_password)
    listener.main1()
    #print("!!!! ",amr_name)
    ld_states_pub1 = LdStatePublisher(listener,amr_name)

    try:
        rclpy.spin(ld_states_pub1)

    except KeyboardInterrupt:
        #listener.close()
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
