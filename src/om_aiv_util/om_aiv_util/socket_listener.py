import io
from turtle import distance
import selectors2 as selectors
import socket
import traceback
import time
import threading
import rclpy
from collections import deque, defaultdict

RECV_BUFFER = 4096

"""
A driver class to handle all input and output communication with ARCL server.

"""
class SocketListener(object):
    def __init__(self, node, addr, port):
        self.selector = None
        self.sock = None
        self.addr = str(addr)
        self.port = int(port)
        self.responses = defaultdict(list)
        self._recv_buffer = b""
        self.lock = threading.Lock()

        self.vel = False
        self.dtg = False
        #self.faults_get_f = False
        #self.query_faults_f = False

        self.node = node

    
    """Given the event mask from selectors, do read or write accordingly.

    param mask:
    The mask to use to decide which event can be executed.
    
    """
    def process_events(self, mask):
        if mask & selectors.EVENT_READ:
            self.read()
        
        self.sort_data()

    """
    Read incoming bytes from the socket and save into receive buffer.
    If there are data received, append into the receiver buffer.
    
    """
    def _read(self):
        try:
            recv_data = self.sock.recv(RECV_BUFFER)
        except io.BlockingIOError:
            pass
        else:
            self._recv_buffer += recv_data

    """
    Read incoming bytes and generate the correct responses.
    
    """
    def read(self):
        self._read()

    """
    Extracts the required string to find from the receive buffer.
    
    Returns:
        str -- The extracted string.
    """
    def sort_data(self):
        while True:
            try:
                newline_char = self._recv_buffer.index(b"\r\n")
            except ValueError:
                return # There is no complete line anymore in buffer, return from here.

            # Extract interested line from receive buffer.
            line = self._recv_buffer[:newline_char]
            self._recv_buffer = self._recv_buffer[newline_char+2:]
            
            try:
                colon = line.index(b':')
            except ValueError:
                key = line
                value = None
                self.responses[key] = [value]
                #self.store(key, value)
            else:
                key = line[:colon]
                value = line[colon+1:].strip()
                if key != b"GetDataStoreFieldValues":
                    self.responses[key] = [value]
                elif key == b"GetDataStoreFieldValues":
                    #print(value[:8])
                    if value[:8] == b"TransVel":
                        key = value[:8]
                        value = value[9:].strip()
                        #print("svalue", value)
                        value = value.decode("utf-8")
                        #print("ssvalue",str(value))
                        self.responses[key] = [value]
                    elif value[:8] == b"Distance":
                        key = value[:8]
                        value = value[17:].strip()
                        value = value.decode("utf-8")
                        #print("dvalue", value)
                        self.responses[key] = [value]
                #self.store(key, value)
                

    # def store(self, key, value):
    #     #node = rclpy.create_node("testing")
    #     self.node.get_logger().info("store....")
    #     print("key ", key)
    #     if key == b"GetDataStoreFieldValues: TransVel":
    #          self.node.get_logger().info("if g ")
    #          if self.vel:
    #              self.responses[key].append(value)
    #              print("vel",value,key)
    #              #node.get_logger().info(str(self.vel))
    #          else:
    #              self.responses[key] = [value]
    #              self.vel = True
    #              #node.get_logger().info(str(self.vel))
    #     elif key == b"EndOfGetDataStoreFieldValues":
    #          self.vel = False

    #     elif key == b"GetDataStoreFieldValues: Distance_to_Goal":
    #          # TODO: Account for values which contains spaces
    #         if self.dtg:
    #             self.responses[key].append(value)
    #         else:
    #             self.responses[key] = [value]
    #             self.dtg = True
    #     elif key == b"EndOfGetDataStoreFieldValues":
    #         self.dtg = False
    #     else:
    #         self.responses[key] = [value]
            
    """
    Retrieve the response associated with the given identifier integer.

    param identifier:
    The identifier integer associated with returned response.
    
    Returns:
        str -- The response string associated with the identifier integer.
    """
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

    """
    # TODO: Add check for failed connection
    Connects the driver to the given address and port.

    param addr:
    Address to connect to.

    param port:
    Port number to connect to.

    """
    def begin(self):
        self.node.get_logger().info("Attempt to listen for incoming on " + str(self.addr) + " at port " + str(self.port))

        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                incoming = (self.addr, self.port)
                sock.bind(incoming)
            except socket.error as e:
                self.node.get_logger().info("Socket listener connection failed" + str(e))
                self.node.get_logger().info("Retrying")
            else:
                break
            finally:
                time.sleep(3)

        sock.listen(1)
        (connection, address) = sock.accept()
        sock.shutdown(socket.SHUT_RDWR)
        sock.close() # Close the initial socket.
        self.node.get_logger().info("Listening to new socket on " + str(address[0]) + " at port " + str(address[1]))
        self.sock = connection
        self.sock.setblocking(False)
        events = selectors.EVENT_READ
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.sock, events, data=self)

    def close(self):
        self.selector.unregister(self.sock)
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()


