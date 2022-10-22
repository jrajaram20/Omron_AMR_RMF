#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import math
import yaml
import json
import time
import copy
import argparse
from rclpy.action import ActionClient

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import RobotState, Location, PathRequest, \
    DockSummary, RobotMode

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry

import numpy as np
from pyproj import Transformer

import socketio

from om_aiv_msg.msg import Location, Status#, Locationrmf, Robotstate
from om_aiv_msg.action import Action
from om_aiv_msg.srv import ArclApi

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

import threading
app = FastAPI()


class Request(BaseModel):
    map_name: Optional[str] = None
    task: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_path_request = None
        self.last_completed_request = None
        self.mode_teleop = False
    #    self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
    #    self.gps_pos = [0, 0]

    # def gps_to_xy(self, gps_json: dict):
    #     svy21_xy = \
    #         self.svy_transformer.transform(gps_json['lat'], gps_json['lon'])
    #     self.gps_pos[0] = svy21_xy[1]
    #     self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_path_request is not None:
            if task_id != self.last_path_request.task_id:
                return False
        return True


class FleetManager(Node):
    def __init__(self, config, nav_path):
        self.debug = False
        self.config = config
        self.fleet_name = self.config["rmf_fleet"]["name"]

        self.gps = False
        self.offset = [0, 0]
        if 'reference_coordinates' in self.config and \
                'offset' in self.config['reference_coordinates']:
            assert len(self.config['reference_coordinates']['offset']) > 1, \
                ('Please ensure that the offset provided is valid.')
        #     self.gps = True
            self.offset = self.config['reference_coordinates']['offset']

        super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.docks = {}  # Map dock name to waypoints

        for robot_name, robot_config in self.config["robots"].items():
            self.robots[robot_name] = State()
        assert(len(self.robots) > 0)

        profile = traits.Profile(geometry.make_final_convex_circle(
            self.config['rmf_fleet']['profile']['footprint']),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        self.vehicle_traits.differential.reversible =\
            self.config['rmf_fleet']['reversible']

        self.sio = socketio.Client()

        # @self.sio.on("/gps")
        # def message(data):
        #     try:
        #         robot = json.loads(data)
        #         robot_name = robot['robot_id']
        #         self.robots[robot_name].gps_to_xy(robot)
        #     except KeyError as e:
        #         self.get_logger().info(f"Malformed GPS Message!: {e}")

        # if self.gps:
        #     while True:
        #         try:
        #             self.sio.connect('http://0.0.0.0:8080')
        #             break
        #         except Exception:
        #             self.get_logger().info(
        #                 f"Trying to connect to sio server at"
        #                 f"http://0.0.0.0:8080..")
        #             time.sleep(1)

        self.create_subscription(
            Status,
            'Rstate',
            self.robot_state_cb,
            10)
         # Action client to navigate the robot
        self._action_client = ActionClient(self, Action, 'action_server')
        
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        # self.path_pub = self.create_publisher(
        #     PathRequest,
        #     'robot_path_requests',
        #     qos_profile=qos_profile_system_default)

        @app.get('/open-rmf/rmf_demos_fm/status/',
                 response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }
            if robot_name is None:
                response['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    response['data']['all_robots'].append(
                        self.get_robot_state(state, robot_name))
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                response['data'] = self.get_robot_state(state, robot_name)
            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_demos_fm/navigate/',
                  response_model=Response)
        async def navigate(robot_name: str, cmd_id: int, dest: Request):
            response = {'success': False, 'msg': ''}
            #self.get_logger().info(f"navigate:::::::::::::::::::, [{dest}]")
            #self.get_logger().info(f"data in fm,,,,,,,,,,,,,,,,, [{dest.data}]")
            if (robot_name not in self.robots or len(dest.destination) < 1):
                return response
            robot = self.robots[robot_name]
            target_x = dest.destination['x']
            target_y = dest.destination['y']
            target_yaw = dest.destination['yaw']
            target_map = "L1"
            target_speed_limit = dest.speed_limit

            #-------------------------------------------------------------------------------------
            # target_x -= self.offset[0]
            # target_y -= self.offset[1]
            #-------------------------------------------------------------------------------------

            t = self.get_clock().now().to_msg()

            path_request = PathRequest()
            robot = self.robots[robot_name]
            cur_x = robot.state.location.x
            cur_y = robot.state.location.y
            cur_yaw = robot.state.location.theta
            cur_loc = robot.state.location
            path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(disp/self.vehicle_traits.linear.nominal_velocity) +\
                int(abs(abs(cur_yaw) - abs(target_yaw)) /
                    self.vehicle_traits.rotational.nominal_velocity)
            t.sec = t.sec + duration
            target_loc = Location()
            #-------------------------------------------------------------------------------------
            #target_loc.t = t
            #-------------------------------------------------------------------------------------
            target_loc.level_name = target_map
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.theta = target_yaw
            target_loc.level_name = "L1"
            if target_speed_limit > 0:
                target_loc.obey_approach_speed_limit = True
                target_loc.approach_speed_limit = target_speed_limit

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            #self.path_pub.publish(path_request)

            if self.debug:
                print(f'Sending navigate request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = target_loc

            #---------------------------------------------------------------------------------------
            x = int(target_x*1000)
            y = int(target_y*1000)
            #self.get_logger().info(f"x, y:, [{x,y,target_yaw}]")
            #theta = int(math.degrees(['target_loc.theta']))
            theta = int(target_loc.theta)
            if (x < -78360 and x > -78560) and (y < 2277 and y > 2077):
                self.send_goal(f"goto one", ["Arrived at one", "Failed going to goal "])
            elif (x < -78360 and x > -78560) and (y < 4712 and y > 4512):
                self.send_goal(f"goto two", ["Arrived at two", "Failed going to goal "])
            elif (x < -79017 and x > -79217) and (y < 90 and y > -90):
                self.send_goal(f"goto home", ["Arrived at home", "Failed going to goal "])
            else:
                self.send_goal(f"gotoPoint {x} {y} {theta}", ["Arrived at point ", "Failed going to goal "])
                self.get_logger().info(f"gotoPointtttttt, [{x} {y} {theta}]")
            #self.send_goal(f"goto goal6",["Arrived at goal6 ", "Failed going to goal "])
            #----------------------------------------------------------------------------------------
            response['success'] = True
            return response

        @app.get('/open-rmf/rmf_demos_fm/stop_robot/',
                 response_model=Response)
        async def stop(robot_name: str, cmd_id: int):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            # Appending the current location twice will effectively tell the
            # robot to stop
            
            #********************************************************************************************
            # path_request.path.append(robot.state.location)
            # path_request.path.append(robot.state.location)
            #********************************************************************************************
            path_request.task_id = str(cmd_id)
            #self.path_pub.publish(path_request)

            if self.debug:
                print(f'Sending stop request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = None

            #********************************************************************************************
            # req = ArclApi.Request()
            # req.command = "stop"
            # req.line_identifier = "Stopped"
            # res = self.client.call(req)
            self.send_goal(f"stop", ["Stopped", "Failed "])
            #self.get_logger().info(f"STOP RESPONSE:, [{res}]")
            #********************************************************************************************

            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_demos_fm/start_task/',
                  response_model=Response)
        async def start_process(robot_name: str, cmd_id: int, task: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or
                    len(task.task) < 1 or
                    task.task not in self.docks):
                return response

            robot = self.robots[robot_name]

            path_request = PathRequest()
            cur_loc = robot.state.location
            cur_x = cur_loc.x
            cur_y = cur_loc.y
            cur_yaw = cur_loc.theta
            previous_wp = [cur_x, cur_y, cur_yaw]
            target_loc = Location()
            path_request.path.append(cur_loc)
            for wp in self.docks[task.task]:
                target_loc = wp
                path_request.path.append(target_loc)
                previous_wp = [wp.x, wp.y, wp.yaw]

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(cmd_id)
            #self.path_pub.publish(path_request)

            if self.debug:
                print(f'Sending process request for {robot_name}: {cmd_id}')
            robot.last_path_request = path_request
            robot.destination = target_loc

            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_demos_fm/toggle_action/',
                  response_model=Response)
        async def toggle_teleop(robot_name: str, mode: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots):
                return response
            # Toggle action mode
            self.robots[robot_name].mode_teleop = mode.toggle
            response['success'] = True
            return response

    def robot_state_cb(self, msg):
        if (msg.name in self.robots):
            robot = self.robots[msg.name]
            #-------------------------------------------------------------------------------------
            # if not robot.is_expected_task_id(msg.task_id) and \
            #         not robot.mode_teleop:
            #     # This message is out of date, so disregard it.
            #     if robot.last_path_request is not None:
            #         # Resend the latest task request for this robot, in case
            #         # the message was dropped.
            #         if self.debug:
            #             print(
            #                 f'Republishing task request for {msg.name}: '
            #                 f'{robot.last_path_request.task_id}, '
            #                 f'because it is currently following {msg.task_id}'
            #             )
            #         self.path_pub.publish(robot.last_path_request)
            #     return
            #-------------------------------------------------------------------------------------
            robot.state = msg
            #self.get_logger().info(f"robot.state:, [{msg}]")
            #-------------------------------------------------------------------------------------
            # Check if robot has reached destination
            # if robot.destination is None:
            #     return
            # self.get_logger().info(f"idleeeeeee:, [{RobotMode.MODE_IDLE}]")
            # self.get_logger().info(f"charging, [{RobotMode.MODE_CHARGING}]")
            # if (
            #     (
            #         msg.mode.mode == RobotMode.MODE_IDLE
            #         or msg.mode.mode == RobotMode.MODE_CHARGING
            #     )
            #     and len(msg.path) == 0
            # ):
            #     robot = self.robots[msg.name]
            #     robot.destination = None
            #     completed_request = int(msg.task_id)
            #     if robot.last_completed_request != completed_request:
            #         if self.debug:
            #             print(
            #                 f'Detecting completed request for {msg.name}: '
            #                 f'{completed_request}'
            #             )
            #     robot.last_completed_request = completed_request
            
            #-------------------------------------------------------------------------------------

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path


    def get_robot_state(self, robot: State, robot_name):
        data = {}
        position = [robot.state.location.x, robot.state.location.y]
        angle = robot.state.location.theta
        data['robot_name'] = robot_name
        #-------------------------------------------------------------------------------------
        data['map_name'] = "L1"
        #-------------------------------------------------------------------------------------

        data['position'] =\
            {'x': position[0], 'y': position[1], 'theta': angle}
        data['battery'] = robot.state.state_of_charge
        duration = format(float(robot.state.time_to_goal),".2f")
        
        #self.get_logger().info(f"duration0000------------------- [{duration}]")
        
        #-------------------------------------------------------------------------------------
        # if (robot.destination is not None
        #         and robot.last_path_request is not None):
        #     destination = robot.destination
        #     cmd_id = int(robot.last_path_request.task_id)
        #     duration = robot.state.time_to_goal
        #     data['destination_arrival'] = {
        #         'cmd_id': cmd_id,
        #         'duration': duration
        #     }
        # else:
        #     data['destination_arrival'] = None    


        # if (robot.destination is not None
        #         and robot.last_path_request is not None):
        #     destination = robot.destination
        #     # remove offset for calculation if using gps coords
        #     if self.gps:
        #         position[0] -= self.offset[0]
        #         position[1] -= self.offset[1]
        #     # calculate arrival estimate
        #     dist_to_target =\
        #         self.disp(position, [destination.x, destination.y])
        #     ori_delta = abs(abs(angle) - abs(destination.yaw))
        #     if ori_delta > np.pi:
        #         ori_delta = ori_delta - (2 * np.pi)
        #     if ori_delta < -np.pi:
        #         ori_delta = (2 * np.pi) + ori_delta
        #     duration = (dist_to_target /
        #                 self.vehicle_traits.linear.nominal_velocity +
        #                 ori_delta /
        #                 self.vehicle_traits.rotational.nominal_velocity)
        #     cmd_id = int(robot.last_path_request.task_id)
        #     data['destination_arrival'] = {
        #         'cmd_id': cmd_id,
        #         'duration': duration
        #     }
        # else:
        #     data['destination_arrival'] = None

        # data['last_completed_request'] = robot.last_completed_request
        # if (
        #     robot.state.mode.mode == RobotMode.MODE_WAITING
        #     or robot.state.mode.mode == RobotMode.MODE_ADAPTER_ERROR
        # ):
        #     # The name of MODE_WAITING is not very intuitive, but the slotcar
        #     # plugin uses it to indicate when another robot is blocking its
        #     # path.
        #     #
        #     # MODE_ADAPTER_ERROR means the robot received a plan that
        #     # didn't make sense, i.e. the plan expected the robot was starting
        #     # very far from its real present location. When that happens we
        #     # should replan, so we'll set replan to true in that case as well.
        #     data['replan'] = True
        # else:
        #     data['replan'] = False
        #-------------------------------------------------------------------------------------
        return data

    def send_goal(self, command, identifier):
        self.goal = Action.Goal()
        self.goal.command = command
        self.get_logger().info(f"command , [{command}]")
        self.goal.identifier = identifier
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(self.goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        #self.get_logger().info(result.res_msg)
        self.previous_navigation_request_result = result.res_msg

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.feed_msg
        #self.get_logger().info(feedback) 

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet manager...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config, args.nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(app,
                host=config['rmf_fleet']['fleet_manager']['ip'],
                port=config['rmf_fleet']['fleet_manager']['port'],
                log_level='warning')


if __name__ == '__main__':
    main(sys.argv)
