#!/usr/bin/env python
from concurrent.futures import thread
from math import fabs
import socket
import time
import sys
import threading
from collections import deque, defaultdict
import rclpy
from std_msgs.msg import String
#from om_aiv_util.socket_listener import *
#from om_aiv_msg.msg import Status, Location
from rclpy.node import Node


# PARAM_IP_ADDR = node.get_parameter("ip_address").get_parameter_value().string_value
# PARAM_PORT = node.get_parameter("port").get_parameter_value().integer_value
# DEFAULT_PASSWD = node.get_parameter("def_arcl_passwd").get_parameter_value().string_value


DEFAULT_SOCKET_TIMEOUT = 10
RECV_BUFFER = 2048
MAX_CONN_RETRY = 5  
CONN_RETRY_SLP = 2
#ROBOT_NAME = "amr1"

# Feedback strings that is sent by ARCL server. We use them to identify the response of the ARCL server.
PASSWD_PROMPT_STRING = "Enter password:"
WELCOME_STRING = "Welcome to the server."
END_OF_CMDS_STRING = "End of commands"
MAX_TRIES_ERROR = "Maximum number of connection retries reached. Are you sure you are connected? Is the password correct?"

# Driver class to connect a socket to the ARCL server.
class ConnectSocket(object):

    def __init__(self, node, ip , port, pswd):#,sock=None):#, passwd=None):
        # Assign all instance attributes
        self.responses = defaultdict(list)
        self.robot_name = "amr1"
        self.ip = str(ip)
        self.port = int(port)
        self.sock = None
        self.passwd = str(pswd)
        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(DEFAULT_SOCKET_TIMEOUT)
            retries = 0
            while retries < MAX_CONN_RETRY:
                try:
                    print("making connection")
                    self.sock.connect((self.ip,self.port))
                    self.login()
                except socket.error as err:
                    retries = retries + 1
                    #rclpy.logger("Socket make connection failed: %s", err)
                    #node.get_logger().info("Socket make connection failed: %s", err)
                    time.sleep(CONN_RETRY_SLP)
                else:
                    break
            if retries == MAX_CONN_RETRY:
                raise RuntimeError(MAX_TRIES_ERROR)
        else:
            print(self.sock)# = sock

    # Attempts to make the socket connection and login into ARCL server with the given password.
    def make_connection(self):
        #print("***",ip)
        self.sock.connect((self.ip,self.port))
        self.login()

    # Attempts to login into ARCL server.
    # NOTE: This is a blocking function if password prompt or login strings are not received from socket.
    def login(self):
        # Look for the login message from ARCL.
        while True:
            try:
                ret = self.sock.recv(RECV_BUFFER)
            except socket.error as err:
                raise err
            else:
                if PASSWD_PROMPT_STRING in ret.decode("utf-8"):
                    break
        self.sendmsg(self.passwd)

        # Check for login success.
        while True:
            try:
                ret = self.sock.recv(RECV_BUFFER)
            except socket.error as err:
                raise err
            else:
                if END_OF_CMDS_STRING in ret.decode("utf-8"):
                    break
    
    def rcvmsg(self):
        re = self.sock.recv(4096)
        #print(re.decode("utf-8"))
        return re.decode("utf-8")

    def sendmsg(self,data):
        self.sock.sendall(bytes(data + "\n", "utf-8"))
        #re = s.recv(2048)
        #print(re.decode("utf-8"))
        
    def get_response(self, key):
        leng = len(self.responses)
        #self.node.get_logger().info("length ")
        #self.node.get_logger().info(str(leng))
        try:
            val = self.responses[key]
            #rint(val+";;;;"+key)
        except KeyError:
            raise
        else:
            if len(val) == 0:
                raise KeyError
            else:
                return self.responses[key]

    def sort_data(self,data):
        while True:
            try:
                newline_char = data.index("\r\n")
            except ValueError:
                return # There is no complete line anymore in buffer, return from here.
            # Extract interested line from receive buffer.
            line = data[:newline_char]
            data = data[newline_char+2:]
            
            try:
                colon = line.index(':')
            except ValueError:
                key = line
                value = None
                self.responses[key] = [value]
                #self.store(key, value)
            else:
                key = line[:colon]
                value = line[colon+1:].strip()
                #print(value)
                if key != "GetDataStoreFieldValues":
                    self.responses[key] = [value]
                elif key == "GetDataStoreFieldValues":
                    if value[:8] == "TransVel":
                        key = value[:8]
                        value = value[9:].strip()
                        self.responses[key] = [value]
                        #print(value)
                    elif value[:16] == "Distance_to_goal":
                        key = value[:16]
                        value = value[17:].strip()
                        self.responses[key] = [value]
                        #print(value)
                    elif value[:6] == "RobotX":
                        key = value[:6]
                        value = value[7:].strip()
                        self.responses[key] = [value]
                        #print(value)
                    elif value[:6] == "RobotY":
                        key = value[:6]
                        value = value[7:].strip()
                        self.responses[key] = [value]
                        #print(value)
                    elif value[:7] == "RobotTh":
                        key = value[:7]
                        value = value[8:].strip()
                        self.responses[key] = [value]
                        #print(value)
                    elif value[:20] == "BatteryStateOfCharge":
                        key = value[:20]
                        value = value[21:].strip()
                        self.responses[key] = [value]
                        #print(value)
                    elif value[:10] == "ModeStatus":
                        key = value[:10]
                        value = value[11:].strip()
                        self.responses[key] = [value]
                        #print(value)

    def task(self):
        while True:
            self.sendmsg("dsfv RobotX"+"\n"+"dsfv RobotY"+"\n"+"dsfv RobotTh"+"\n"+"dsfv modestatus"+"\n"+"dsfv transvel"+"\n"+"dsfv distance_to_goal"+"\n"+"dsfv batterystateofcharge")
            time.sleep(0.5)
            r = ""
            r = self.rcvmsg()
            self.sort_data(r)
            #print(r)

    def main1(self):
        #self.make_connection()
        self.sock.setblocking(0)
        thread = threading.Thread(target = self.task)
        thread.start()
        # while True:
        #     self.sendmsg("dsfv RobotX"+"\n"+"dsfv RobotY"+"\n"+"dsfv RobotTh"+"\n"+"dsfv modestatus"+"\n"+"dsfv transvel"+"\n"+"dsfv distance_to_goal"+"\n"+"dsfv batterystateofcharge")
        #     time.sleep(0.5)
        #     r = ""
        #     r = self.rcvmsg()
        #     self.sort_data(r)
            #print(r)



if __name__ == "__main__":
    c = ConnectSocket()
    c.main1()

