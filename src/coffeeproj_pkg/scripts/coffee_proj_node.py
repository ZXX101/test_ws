#!/usr/bin/env python3
"""
coffee_proj_node.py
===================
主节点：整合MQTT通信与飞行控制（模块化版本）

此文件为ROS节点主入口，负责：
- 初始化ROS节点和MAVROS订阅
- MQTT通信管理
- 任务接收和状态机触发
- 遥测数据发布
"""

from __future__ import annotations

import math
import os
import sys
import threading

import rospy

_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from utils import (
    FlightState,
    gps_to_enu,
    gps_to_enu_xy,
    enu_to_gps_xy,
    yaw_from_quaternion,
    distance_3d,
    now_ms,
)
from flight_controller import FlightController
from mqtt_client import MqttClient
from state_machine import TaskStateMachine
from zeroone_rtk import ZerooneRtk

from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest


class CoffeeProjNode:
    """
    主节点类

    负责：
    - 初始化ROS节点和MAVROS订阅
    - MQTT通信管理
    - 任务接收和状态机触发
    - 遥测数据发布
    """

    def __init__(self):
        """
        初始化主节点

        包括：
        - ROS节点初始化
        - MAVROS话题订阅（状态、位置、速度、GPS、电池）
        - MQTT客户端初始化
        - 飞行控制器初始化
        - 状态机初始化和启动
        - 遥测定时器
        """
        rospy.init_node('coffee_proj_node', anonymous=False)

        self.drone_id = rospy.get_param('~drone_id', 'drone_001')
        self.telemetry_rate = rospy.get_param('~telemetry_rate', 50.0)

        self._mav_state = State()
        self._local_pose = PoseStamped()
        self._local_vel = TwistStamped()
        self._global_pos = NavSatFix()
        self._battery = BatteryState()
        self._gps_origin = GeoPointStamped()
        self._relative_alt = 0.0

        rospy.Subscriber(
            rospy.get_param('~mavros_state', '/mavros/state'),
            State, self._cb_state, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param('~mavros_local_pose', '/mavros/local_position/pose'),
            PoseStamped, self._cb_local_pose, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param('~mavros_local_vel', '/mavros/local_position/velocity_local'),
            TwistStamped, self._cb_local_vel, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param('~mavros_global_pos', '/mavros/global_position/global'),
            NavSatFix, self._cb_global_pos, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param('~mavros_rel_alt', '/mavros/global_position/rel_alt'),
            Float64, self._cb_global_rel_alt, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param('~mavros_battery', '/mavros/battery'),
            BatteryState, self._cb_battery, queue_size=1
        )
        self._sgpsor_pub = rospy.Publisher(
            rospy.get_param('~mavros_setgps_origin', '/mavros/global_position/set_gp_origin'),
            GeoPointStamped, queue_size=1
        )

        self.zeroone_rtk = ZerooneRtk(self)

        self.mqtt_client = MqttClient(self)
        self.flight_controller = FlightController(self)

        self.state_machine = TaskStateMachine(self, self.flight_controller, self.mqtt_client)
        self.state_machine.start()

        self.pubTimer = rospy.Timer(
            rospy.Duration(1.0 / self.telemetry_rate),
            self._publish_telemetry
        )

        rospy.loginfo("[CoffeeProj] Node ready. drone_id=%s", self.drone_id)

    def _cb_state(self, msg):
        """MAVROS状态回调"""
        self._mav_state = msg

    def _cb_local_pose(self, msg):
        """本地位置回调"""
        self._local_pose = msg

    def _cb_local_vel(self, msg):
        """本地速度回调"""
        self._local_vel = msg

    def _cb_global_pos(self, msg):
        """GPS全局位置回调"""
        self._global_pos = msg

    def _cb_global_rel_alt(self, msg):
        """GPS相对高度回调"""
        self._relative_alt = msg.data

    def _cb_battery(self, msg):
        """电池状态回调"""
        self._battery = msg

    def _cb_gps_origin(self, msg):
        """GPS原点回调"""
        self._gps_origin = msg

    def _publish_telemetry(self, event):
        """
        定时发布遥测数据

        Args:
            event: ROS定时器事件对象

        发布内容包括：
        - 任务ID、状态
        - GPS位置（lat, lng, alt）
        - 航向角、速度
        - 电池电量、GPS状态、飞行模式

        Note: 添加异常处理防止定时器因异常停止
        """
        try:
            if not self.mqtt_client.is_connected():
                rospy.logdebug("[CoffeeProj] MQTT not connected, skip telemetry")
                return

            rospy.logdebug("[CoffeeProj] MQTT connected, preparing telemetry data")

            state = self.state_machine.get_state()

            if state in [FlightState.FAILED, FlightState.ERROR, FlightState.ABORTED]:
                status = "ERROR"
            elif state in [
                FlightState.LOADED,
                FlightState.TAKEOFF,
                FlightState.FLYING,
                FlightState.ARRIVED,
                FlightState.LANDING,
                FlightState.LANDED,
                FlightState.UNLOADED,
                FlightState.RTL_FLYING,
                FlightState.RTL_LANDING,
            ]:
                status = "FLY"
            else:
                status = "IDLE"

            vx = self._local_vel.twist.linear.x
            vy = self._local_vel.twist.linear.y
            speed = math.sqrt(vx * vx + vy * vy)

            heading = yaw_from_quaternion(self._local_pose.pose.orientation)

            gps_ok = self._global_pos.status.status >= 0 if self._global_pos.status else False
            gps_status = "ok" if gps_ok else "no_fix"

            bat_pct = self._battery.percentage * 100.0 if self._battery.percentage >= 0 else -1.0

            task_data = self.state_machine.get_task_data()
            task_id = task_data.get('task_id') if task_data else None

            rospy.logdebug("[CoffeeProj] Data prepared: lat=%.6f, lng=%.6f, alt=%.1f",
                          self._global_pos.latitude, self._global_pos.longitude, self._relative_alt)

            data = {
                'taskId': task_id,
                'status': status,
                'lat': self._global_pos.latitude,
                'lng': self._global_pos.longitude,
                'alt': self._relative_alt,
                'heading': round(heading, 1),
                'speed': round(speed, 2),
                'battery': round(bat_pct, 1),
                'gpsStatus': gps_status,
                'flightMode': self._mav_state.mode
            }

            rospy.logdebug("[CoffeeProj] Calling mqtt_client.publish_telemetry")
            self.mqtt_client.publish_telemetry(data)
            rospy.logdebug("[CoffeeProj] Telemetry published successfully")
        except Exception as e:
            rospy.logerr("[CoffeeProj] Telemetry publish error: %s", e)
            import traceback
            rospy.logerr("[CoffeeProj] Traceback: %s", traceback.format_exc())

    def handle_task(self, payload):
        """
        处理MQTT任务报文

        Args:
            payload: 任务报文JSON字典

        任务类型：
        - assign: 分配任务，解析任务数据并设置到状态机
        - cancel: 取消任务，触发状态机进入CANCELLED状态
        """
        type = payload.get('type', '')

        rospy.loginfo("[CoffeeProj] Task received: type=%s", type)

        if type == 'assign':
            self._handle_assign(payload)
        elif type == 'cancel':
            self._handle_cancel(payload)
        else:
            rospy.logwarn("[CoffeeProj] Unknown task type: %s", type)

    def _handle_assign(self, payload):
        """
        处理任务分配报文

        Args:
            payload: 任务分配报文

        流程：
        1. 解析任务数据（waypoints, GPS等）
        2. 设置任务数据到状态机
        3. 状态机进入LOADED状态
        4. 等待takeoff指令触发执行

        阶段说明：
        - 任务数据已加载，等待GCS发送takeoff指令
        """
        task_id = payload.get('taskId', '')
        task = payload.get('task', {})

        waypoint_list = task.get('waypointList', {}).get('wayPoints', [])
        rtl_waypoint_list = task.get('rtlWaypointList', {}).get('wayPoints', [])
        rtl_type = payload.get('rtlType', 'reverse')

        home = task.get('home', {})
        home_gps = (
            home.get('lat', 0),
            home.get('lng', 0),
            home.get('cruiseHeight', 0)
        )

        if home_gps[0] != 0 and home_gps[1] != 0:
            gp_origin_msg = GeoPointStamped()
            gp_origin_msg.position.latitude = home_gps[0]
            gp_origin_msg.position.longitude = home_gps[1]
            self._sgpsor_pub.publish(gp_origin_msg)
            rospy.loginfo("[CoffeeProj] Updated GPS origin to lat=%.6f, lng=%.6f", home_gps[0], home_gps[1])

        dest = task.get('dest', {})
        dest_gps = (
            dest.get('lat', 0),
            dest.get('lng', 0),
            dest.get('alt', 0)
        )

        cruise_height = task.get('cruiseHeight', 100)

        for waypoint in waypoint_list:
            waypoint['alt'] = cruise_height

        for waypoint in rtl_waypoint_list:
            waypoint['alt'] = cruise_height

        order_id = payload.get('orderId', '')

        task_data = {
            'task_id': task_id,
            'order_id': order_id,
            'waypoint_list': waypoint_list,
            'rtl_waypoint_list': rtl_waypoint_list,
            'rtl_type': rtl_type,
            'cruise_speed': task.get('cruise_speed', 5.0),
            'cruise_height': cruise_height,
            'home_gps': home_gps,
            'dest_gps': dest_gps
        }

        rospy.loginfo("[CoffeeProj] Task assigned: %s", task_id)
        rospy.loginfo("[CoffeeProj] Home GPS: lat=%.6f lng=%.6f alt=%.2f",
                      home_gps[0], home_gps[1], home_gps[2])
        rospy.loginfo("[CoffeeProj] Dest GPS: lat=%.6f lng=%.6f alt=%.2f",
                      dest_gps[0], dest_gps[1], dest_gps[2])
        rospy.loginfo("[CoffeeProj] Waypoints: %d, RTL waypoints: %d",
                      len(waypoint_list), len(rtl_waypoint_list))
        rospy.loginfo("[CoffeeProj] Waiting for takeoff command...")

        self.state_machine.set_task_data(task_data)
        self.state_machine.set_state(FlightState.LOADED)
        self.mqtt_client.publish_status(task_id, FlightState.LOADED, "Task assigned, waiting for takeoff", order_id)

    def _handle_cancel(self, payload):
        """
        处理任务取消报文

        Args:
            payload: 取消报文

        清除任务状态，状态机进入CANCELLED状态
        """
        task_id = payload.get('taskId', '')
        order_id = payload.get('orderId', '')

        rospy.loginfo("[CoffeeProj] Task cancelled: %s", task_id)

        self.state_machine.set_state(FlightState.CANCELLED)

    def handle_command(self, payload):
        """
        处理MQTT控制指令

        Args:
            payload: 指令报文JSON字典

        指令类型：
        - takeoff: 起飞指令，触发状态机起飞
        - land: 降落指令，触发状态机降落
        - rtl: 返航测试指令（仅用于仿真测试）
        """
        type = payload.get('type', '')

        rospy.loginfo("[CoffeeProj] Command received: type=%s", type)

        if type == 'takeoff':
            self._handle_command_takeoff(payload)
        elif type == 'land':
            self._handle_command_land(payload)
        elif type == 'rtl':
            if self.state_machine.get_state() == FlightState.UNLOADED:
                self._handle_command_takeoff(payload)
            else:
                self._handle_command_rtltest(payload)
        else:
            rospy.logwarn("[CoffeeProj] Unknown command type: %s", type)

    def _handle_command_takeoff(self, payload):
        """
        处理起飞指令

        Args:
            payload: 起飞指令报文

        两种场景：
        1. 状态机在LOADED状态 → 触发起飞，开始任务
        2. 状态机在UNLOADED状态 → 触发起飞，开始返航
        3. 其他状态 → 执行简单起飞（无任务）
        """
        current_state = self.state_machine.get_state()

        if current_state == FlightState.LOADED:
            rospy.loginfo("[CoffeeProj] Takeoff command received, starting task")
            self.state_machine.request_takeoff()
        elif current_state == FlightState.UNLOADED:
            rospy.loginfo("[CoffeeProj] Takeoff command received, starting return flight")
            self.state_machine.request_takeoff()
        else:
            rospy.loginfo("[CoffeeProj] Simple takeoff command (no task)")
            t = threading.Thread(target=self.flight_controller.takeoff, daemon=True)
            t.start()

    def _handle_command_land(self, payload):
        """
        处理降落指令

        Args:
            payload: 降落指令报文

        两种场景：
        1. 状态机在ARRIVED状态（悬停等待） → 触发降落
        2. 其他状态 → 执行简单降落
        """
        current_state = self.state_machine.get_state()

        if current_state == FlightState.ARRIVED:
            rospy.loginfo("[CoffeeProj] Land command received while hovering")
            self.state_machine.request_land()
        else:
            rospy.loginfo("[CoffeeProj] Simple land command (no task)")
            t = threading.Thread(target=self.flight_controller.land, daemon=True)
            t.start()

    def _handle_command_rtltest(self, payload):
        """
        处理返航测试指令

        Args:
            payload: 返航测试指令报文

        仅用于仿真测试，可随时执行，打断当前任务

        安全说明：
        - 实飞环境不应使用此功能
        - 会中断当前任务并重置状态
        """
        rospy.loginfo("[CoffeeProj] RTL test command received")

        self.state_machine.request_abort()
        while self.state_machine.get_state() not in [FlightState.ABORTED, FlightState.FAILED, FlightState.ERROR, FlightState.IDLE]:
            rospy.loginfo("[CoffeeProj] Waiting for state machine to abort current task...")
            rospy.sleep(1.0)

        currentPos = [self._global_pos.longitude, self._global_pos.latitude]
        homePos = [self._gps_origin.position.longitude, self._global_pos.latitude]

        t = threading.Thread(target=self.flight_controller.return_to_home, daemon=True)
        t.start()

    def run(self):
        """
        启动节点主循环

        阻塞运行直到节点关闭
        """
        rospy.spin()

        self.state_machine.stop()
        if self.zeroone_rtk:
            self.zeroone_rtk.shutdown()
        self.mqtt_client.shutdown()


if __name__ == '__main__':
    node = CoffeeProjNode()
    node.run()