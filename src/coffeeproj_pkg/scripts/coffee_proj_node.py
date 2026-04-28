#!/usr/bin/env python3
"""
coffee_proj_node.py
===================
主节点：整合MQTT通信与飞行控制（单文件版本）
"""

import math
import json
import time
import rospy
import threading
import paho.mqtt.client as mqtt
import ssl
from enum import IntEnum
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from pygeodesy.geoids import GeoidPGM

# =============================================================================
# 工具函数
# =============================================================================

_EARTH_R = 6_371_000.0

def gps_to_enu(lat, lon, alt, home_lat, home_lon, home_alt):
    """
    将GPS坐标转换为ENU本地坐标（东-北-上）
    
    Args:
        lat: 目标点纬度（度）
        lon: 目标点经度（度）
        alt: 目标点高度（米）
        home_lat: 原点纬度（度）
        home_lon: 原点经度（度）
        home_alt: 原点高度（米）
    
    Returns:
        (x, y, z): ENU坐标（米），x为东向，y为北向，z为向上
    """
    dlat = math.radians(lat - home_lat)
    dlon = math.radians(lon - home_lon)
    x = _EARTH_R * dlon * math.cos(math.radians(home_lat))
    y = _EARTH_R * dlat
    z = alt - home_alt
    return x, y, z

def gps_to_enu_xy(lat, lon, home_lat, home_lon):
    """
    将GPS坐标的水平分量转换为ENU坐标（只转换lat/lng -> x/y）
    
    Args:
        lat: 目标点纬度（度）
        lon: 目标点经度（度）
        home_lat: 原点纬度（度）
        home_lon: 原点经度（度）
    
    Returns:
        (x, y): ENU水平坐标（米），x为东向，y为北向
    """
    dlat = math.radians(lat - home_lat)
    dlon = math.radians(lon - home_lon)
    x = _EARTH_R * dlon * math.cos(math.radians(home_lat))
    y = _EARTH_R * dlat
    return x, y


def enu_to_gps_xy(x, y, home_lat, home_lon):
    """
    Convert ENU horizontal coordinates to GPS coordinates (x/y -> lat/lon)
    
    Args:
        x: East position (meters)
        y: North position (meters)
        home_lat: Origin latitude (degrees)
        home_lon: Origin longitude (degrees)
    
    Returns:
        (lat, lon): GPS coordinates (degrees)
    """
    lat = home_lat + math.degrees(y / _EARTH_R)
    lon = home_lon + math.degrees(x / (_EARTH_R * math.cos(math.radians(home_lat))))
    return lat, lon
def yaw_from_quaternion(q):
    """
    从四元数提取偏航角
    
    Args:
        q: 四元数对象（geometry_msgs/Quaternion）
    
    Returns:
        偏航角（度），范围 -180 到 180
    """
    yaw_rad = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )
    return math.degrees(yaw_rad)

def distance_3d(pose: PoseStamped, x_b, y_b, z_b):
    """
    计算当前位置与目标点的三维距离
    
    Args:
        pose: 当前位置的PoseStamped消息
        x_b: 目标点x坐标（米）
        y_b: 目标点y坐标（米）
        z_b: 目标点z坐标（米）
    
    Returns:
        三维距离（米）
    """
    dx = pose.pose.position.x - x_b
    dy = pose.pose.position.y - y_b
    dz = pose.pose.position.z - z_b
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def now_ms():
    """
    获取当前时间戳（毫秒）
    
    Returns:
        当前Unix时间戳（毫秒）
    """
    return int(time.time() * 1000)


# =============================================================================
# 任务状态枚举
# =============================================================================

class FlightState(IntEnum):
    """
    飞行任务状态枚举
    
    定义任务执行的完整生命周期状态
    """
    IDLE = 0          # 空闲，无任务
    PRE_CHECK = 1     # 预检查（预留）
    LOADING = 2       # 加载任务数据
    LOADED = 3        # 任务已加载，等待起飞指令
    TAKEOFF = 4       # 起飞中
    FLYING = 5        # 飞往目的地
    ARRIVED = 6       # 到达目的地，悬停等待降落指令
    LANDING = 7       # 降落中
    LANDED = 8        # 已降落，卸货中
    UNLOADED = 9      # 卸货完成，等待返航起飞指令
    RTL_FLYING = 10   # 返航飞行中
    RTL_LANDING = 11  # 返航降落中
    COMPLETED = 12    # 任务完成
    CANCELLED = 13    # 任务取消
    FAILED = 14       # 任务失败
    ERROR = 15        # 错误状态
    ABORTED = 16      # 紧急中断（新增）


# =============================================================================
# 状态机类
# =============================================================================

class TaskStateMachine:
    """
    任务状态机
    
    负责：
    - 状态管理和转换
    - 状态执行逻辑
    - 安全检查（飞行模式监控）
    - 中断处理
    
    状态转换图：
    IDLE → LOADED → TAKEOFF → FLYING → ARRIVED → LANDING → LANDED → UNLOADED
                              ↓                      ↓
                         RTL_FLYING ←───────────── RTL起飞
                              ↓
                         RTL_LANDING → COMPLETED
    
    安全机制：
    - 检测到飞行模式变化（非OFFBOARD）时自动进入ABORTED状态
    - ABORTED状态下停止所有设定点发布
    """
    
    def __init__(self, node, flight_controller, mqtt_client):
        """
        初始化状态机
        
        Args:
            node: 主节点对象（CoffeeProjNode）
            flight_controller: 飞行控制器
            mqtt_client: MQTT客户端
        """
        self.node = node
        self.flight_controller = flight_controller
        self.mqtt_client = mqtt_client
        
        self._state = FlightState.IDLE
        self._state_lock = threading.Lock()
        
        self._task_data = None
        self._task_data_lock = threading.Lock()
        
        self._abort_requested = False
        self._land_requested = False
        self._takeoff_requested = False
        self._is_return_phase = False
        
        self._running = False
        self._state_thread = None
        
        self._hover_rate = rospy.Rate(10)
        self._ground_wait_rate = rospy.Rate(2)
    
    def get_state(self):
        """
        获取当前状态
        
        Returns:
            FlightState枚举值
        """
        with self._state_lock:
            return self._state
    
    def set_state(self, state):
        """
        设置状态
        
        Args:
            state: FlightState枚举值
        """
        with self._state_lock:
            old_state = self._state
            self._state = state
            rospy.loginfo("[StateMachine] State transition: %s → %s", 
                          old_state.name if hasattr(old_state, 'name') else old_state,
                          state.name if hasattr(state, 'name') else state)
    
    def set_task_data(self, task_data):
        """
        设置任务数据
        
        Args:
            task_data: 任务数据字典，包含：
                - task_id: 任务ID
                - order_id: 订单ID
                - waypoint_list: 航点列表
                - rtl_waypoint_list: 返航航点列表
                - rtl_type: 返航类型
                - cruise_speed: 巡航速度
                - cruise_height: 巡航高度
                - home_gps: Home点GPS
                - dest_gps: 目的地GPS
        """
        with self._task_data_lock:
            self._task_data = task_data
    
    def get_task_data(self):
        """
        获取任务数据
        
        Returns:
            任务数据字典，或None
        """
        with self._task_data_lock:
            return self._task_data
    
    def request_abort(self):
        """
        请求紧急中断
        
        设置中断标志，状态机将进入ABORTED状态
        """
        self._abort_requested = True
        self.flight_controller.set_abort(True)
        rospy.logwarn("[StateMachine] Abort requested!")
    
    def request_land(self):
        """
        请求降落
        
        在ARRIVED状态触发降落
        """
        self._land_requested = True
        rospy.loginfo("[StateMachine] Land requested")
    
    def request_takeoff(self):
        """
        请求起飞
        
        在LOADED或UNLOADED状态触发起飞
        """
        self._takeoff_requested = True
        rospy.loginfo("[StateMachine] Takeoff requested")
    
    def clear_requests(self):
        """
        清除所有请求标志
        """
        self._abort_requested = False
        self._land_requested = False
        self._takeoff_requested = False
    
    def is_offboard(self):
        """
        检查是否在OFFBOARD模式
        
        Returns:
            True: OFFBOARD模式, False: 其他模式
        """
        return self.flight_controller.is_offboard()
    
    def check_flight_mode(self):
        """
        检查飞行模式
        
        如果不在OFFBOARD模式且任务正在执行中，触发中断
        
        Note: During TAKEOFF state start, OFFBOARD mode is not yet set,
              so we skip the check at the beginning of takeoff.
        
        Returns:
            True: 模式正常, False: 模式异常（已触发中断）
        """
        current_state = self.get_state()
        
        active_states = [
            FlightState.FLYING,
            FlightState.ARRIVED,
            FlightState.LANDING,
            FlightState.RTL_FLYING,
            FlightState.RTL_LANDING,
        ]
        
        # Skip check during TAKEOFF - takeoff function will switch to OFFBOARD
        # Only check after takeoff has started sending setpoints
        
        if current_state in active_states and not self.is_offboard():
            rospy.logwarn("[StateMachine] Flight mode changed to %s, aborting task!",
                          self.flight_controller._mav_state.mode)
            self.request_abort()
            return False
        
        return True
    
    def start(self):
        """
        启动状态机
        
        开始状态循环线程
        """
        if self._running:
            rospy.logwarn("[StateMachine] Already running")
            return
        
        self._running = True
        self._state_thread = threading.Thread(target=self._state_loop, daemon=True)
        self._state_thread.start()
        rospy.loginfo("[StateMachine] Started")
    
    def stop(self):
        """
        停止状态机
        """
        self._running = False
        rospy.loginfo("[StateMachine] Stopped")
    
    def _state_loop(self):
        """
        状态机主循环
        
        根据当前状态执行对应的状态处理函数
        """
        while self._running and not rospy.is_shutdown():
            current_state = self.get_state()
            
            if self._abort_requested and current_state != FlightState.ABORTED:
                self._handle_aborted()
                continue
            
            try:
                self._handle_state(current_state)
            except Exception as e:
                rospy.logerr("[StateMachine] Error in state %s: %s", 
                             current_state.name if hasattr(current_state, 'name') else current_state, e)
                self.set_state(FlightState.ERROR)
            
            rospy.sleep(0.05)
    
    def _handle_state(self, state):
        """
        状态处理函数
        
        Args:
            state: 当前状态
        """
        handlers = {
            FlightState.IDLE: self._handle_idle,
            FlightState.LOADED: self._handle_loaded,
            FlightState.TAKEOFF: self._handle_takeoff,
            FlightState.FLYING: self._handle_flying,
            FlightState.ARRIVED: self._handle_arrived,
            FlightState.LANDING: self._handle_landing,
            FlightState.LANDED: self._handle_landed,
            FlightState.UNLOADED: self._handle_unloaded,
            FlightState.RTL_FLYING: self._handle_rtl_flying,
            FlightState.RTL_LANDING: self._handle_rtl_landing,
            FlightState.COMPLETED: self._handle_completed,
            FlightState.CANCELLED: self._handle_cancelled,
            FlightState.FAILED: self._handle_failed,
            FlightState.ERROR: self._handle_error,
            FlightState.ABORTED: self._handle_aborted,
        }
        
        handler = handlers.get(state, self._handle_idle)
        handler()
    
    def _handle_idle(self):
        """
        空闲状态处理
        
        无任务时等待，不做任何操作
        """
        pass
    
    def _handle_loaded(self):
        """
        已加载状态处理
        
        等待起飞指令
        
        阶段说明：
        - 任务数据已加载，等待GCS发送takeoff指令
        - 收到takeoff指令后先切换OFFBOARD模式
        - 再进入TAKEOFF状态执行起飞
        """
        if self._takeoff_requested:
            self._takeoff_requested = False
            task_data = self.get_task_data()
            
            if task_data:
                # ============================================================================
                # 先切换到OFFBOARD模式（确保起飞前模式正确）
                # ============================================================================
                if not self.flight_controller.is_offboard():
                    rospy.loginfo("[StateMachine] Setting OFFBOARD mode before takeoff")
                    if not self.flight_controller.set_mode('OFFBOARD'):
                        rospy.logerr("[StateMachine] Failed to set OFFBOARD mode")
                        self.mqtt_client.publish_status(
                            task_data['task_id'],
                            FlightState.FAILED,
                            "Failed to set OFFBOARD mode",
                            task_data['order_id']
                        )
                        self.set_state(FlightState.FAILED)
                        return
                    
                    rospy.sleep(0.5)  # 等待模式切换生效
                
                # ============================================================================
                # 进入TAKEOFF状态
                # ============================================================================
                self.mqtt_client.publish_status(
                    task_data['task_id'], 
                    FlightState.TAKEOFF, 
                    "Takeoff from home", 
                    task_data['order_id']
                )
                self.set_state(FlightState.TAKEOFF)
            else:
                rospy.logerr("[StateMachine] No task data for takeoff")
                self.set_state(FlightState.IDLE)
    
    def _handle_takeoff(self):
        """
        起飞状态处理
        
        阶段说明：
        - 从home起飞到takeoff_alt高度
        - 然后升高到巡航高度（如果需要）
        - 完成后进入FLYING状态
        """
        task_data = self.get_task_data()
        if not task_data:
            rospy.logerr("[StateMachine] No task data")
            self.set_state(FlightState.FAILED)
            return
        
        if not self.check_flight_mode():
            return
        
        takeoff_alt = self.flight_controller.takeoff_alt
        cruise_height = task_data['cruise_height']
        
        rospy.loginfo("[StateMachine] Takeoff to %.1f m", takeoff_alt)
        
        if not self.flight_controller.takeoff(takeoff_alt):
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.FAILED,
                "Takeoff failed",
                task_data['order_id']
            )
            self.set_state(FlightState.FAILED)
            return
        
        rospy.loginfo("[StateMachine] Takeoff complete at %.1f m", takeoff_alt)
        
        # ============================================================================
        # 升高到巡航高度（使用位置控制，而非二次takeoff）
        # ============================================================================
        if cruise_height and cruise_height > takeoff_alt:
            rospy.loginfo("[StateMachine] Climbing to cruise height %.1f m", cruise_height)
            
            if not self.check_flight_mode():
                return
            
            if not self.flight_controller.goto_altitude(cruise_height):
                self.mqtt_client.publish_status(
                    task_data['task_id'],
                    FlightState.FAILED,
                    "Failed to reach cruise height",
                    task_data['order_id']
                )
                self.set_state(FlightState.FAILED)
                return
            
            rospy.loginfo("[StateMachine] Reached cruise height %.1f m", cruise_height)
        
        self.mqtt_client.publish_status(
            task_data['task_id'],
            FlightState.FLYING,
            "Flying waypoints to dest",
            task_data['order_id']
        )
        self.set_state(FlightState.FLYING)
    
    def _handle_flying(self):
        """
        飞行状态处理
        
        阶段说明：
        - 飞往waypointList途径航点
        - 飞往dest目的地
        - 完成后进入ARRIVED状态
        """
        task_data = self.get_task_data()
        if not task_data:
            rospy.logerr("[StateMachine] No task data")
            self.set_state(FlightState.FAILED)
            return
        
        waypoint_list = task_data['waypoint_list']
        home_gps = task_data['home_gps']
        dest_gps = task_data['dest_gps']
        cruise_height = task_data['cruise_height']
        
        # ============================================================================
        # 飞往waypointList（途径航点）
        # ============================================================================
        if waypoint_list:
            rospy.loginfo("[StateMachine] Flying %d waypoints", len(waypoint_list))
            
            if not self.check_flight_mode():
                return
            
            if not self.flight_controller.execute_waypoints(waypoint_list, home_gps):
                self.mqtt_client.publish_status(
                    task_data['task_id'],
                    FlightState.FAILED,
                    "Waypoint execution failed",
                    task_data['order_id']
                )
                self.set_state(FlightState.FAILED)
                return
        
        # ============================================================================
        # 飞往dest（目标目的地）
        # ============================================================================
        dest_point = {'lat': dest_gps[0], 'lng': dest_gps[1], 'alt': cruise_height}
        rospy.loginfo("[StateMachine] Flying to destination")
        
        if not self.check_flight_mode():
            return
        
        if not self.flight_controller.execute_waypoints([dest_point], home_gps):
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.FAILED,
                "Failed to reach destination",
                task_data['order_id']
            )
            self.set_state(FlightState.FAILED)
            return
        
        self.mqtt_client.publish_status(
            task_data['task_id'],
            FlightState.ARRIVED,
            "Arrived at destination, hovering for land command",
            task_data['order_id']
        )
        self.set_state(FlightState.ARRIVED)
    
    def _handle_arrived(self):
        """
        到达状态处理
        
        阶段说明：
        - 到达dest/home，悬停等待降落指令
        - 检查飞行模式，非OFFBOARD时触发中断
        - 收到land指令后进入LANDING或RTL_LANDING状态
        
        返程判断：
        - _is_return_phase=False → 在dest，进入LANDING
        - _is_return_phase=True → 在home，进入RTL_LANDING
        """
        task_data = self.get_task_data()
        
        location = "home" if self._is_return_phase else "dest"
        rospy.loginfo("[StateMachine] Hovering at %s, waiting for land command...", location)
        
        while self._running and not rospy.is_shutdown() and not self._land_requested:
            if self._abort_requested:
                self._handle_aborted()
                return
            
            if not self.check_flight_mode():
                return
            
            self.flight_controller.hover()
            self._hover_rate.sleep()
        
        if self._land_requested:
            self._land_requested = False
            
            if self._is_return_phase:
                rospy.loginfo("[StateMachine] Land command received, landing at home")
                self.mqtt_client.publish_status(
                    task_data['task_id'],
                    FlightState.RTL_LANDING,
                    "Landing at home",
                    task_data['order_id']
                )
                self.set_state(FlightState.RTL_LANDING)
            else:
                rospy.loginfo("[StateMachine] Land command received, landing at dest")
                self.mqtt_client.publish_status(
                    task_data['task_id'],
                    FlightState.LANDING,
                    "Landing at destination",
                    task_data['order_id']
                )
                self.set_state(FlightState.LANDING)
    
    def _handle_landing(self):
        """
        降落状态处理
        
        阶段说明：
        - 在dest降落
        - 完成后进入LANDED状态
        """
        task_data = self.get_task_data()
        
        # ============================================================================
        # 确保返程标志为False（去程阶段）
        # ============================================================================
        self._is_return_phase = False
        
        if not self.check_flight_mode():
            return
        
        if not self.flight_controller.land():
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.FAILED,
                "Landing failed",
                task_data['order_id']
            )
            self.set_state(FlightState.FAILED)
            return
        
        self.mqtt_client.publish_status(
            task_data['task_id'],
            FlightState.LANDED,
            "Landed at destination, unloading",
            task_data['order_id']
        )
        self.set_state(FlightState.LANDED)
    
    def _handle_landed(self):
        """
        已降落状态处理
        
        阶段说明：
        - 卸货（在地面等待unload_timeS秒）
        - 完成后进入UNLOADED状态
        """
        task_data = self.get_task_data()
        
        rospy.loginfo("[StateMachine] Unloading for %.1f seconds...", 
                      self.flight_controller.unload_timeS)
        rospy.sleep(self.flight_controller.unload_timeS)
        
        self.mqtt_client.publish_status(
            task_data['task_id'],
            FlightState.UNLOADED,
            "Unloaded cargo, waiting for takeoff command",
            task_data['order_id']
        )
        self.set_state(FlightState.UNLOADED)
    
    def _handle_unloaded(self):
        """
        卸货完成状态处理
        
        阶段说明：
        - 在地面等待返航起飞指令
        - 收到takeoff指令后进入RTL_FLYING状态
        """
        task_data = self.get_task_data()
        
        rospy.loginfo("[StateMachine] On ground at dest, waiting for takeoff command...")
        
        #睡眠等待起飞指令，同时监控中断和飞行模式变化
        #CoffeeProjNode收到后回调用TaskStateMachine的request_takeoff，这会把_takeoff_requested设为true，导致跳出while循环
        while self._running and not rospy.is_shutdown() and not self._takeoff_requested:
            rospy.loginfo("[StateMachine] On ground at dest, waiting for takeoff command...")
            if self._abort_requested:
                self._handle_aborted()
                return
            
            self._ground_wait_rate.sleep()
        
        if self._takeoff_requested:
            self._takeoff_requested = False
            
            rospy.loginfo("[StateMachine] Takeoff command received, starting return flight")
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.RTL_FLYING,
                "Takeoff for return",
                task_data['order_id']
            )
            self.set_state(FlightState.RTL_FLYING)
    
    def _handle_rtl_flying(self):
        """
        返航飞行状态处理
        
        阶段说明：
        - 从dest起飞到takeoff_alt高度
        - 升高到巡航高度（如果需要）
        - 飞返程waypoints（根据rtl_type决定航线）
        - 飞往home（商家起点）
        - 完成后进入ARRIVED状态（在home悬停）
        """
        task_data = self.get_task_data()
        if not task_data:
            rospy.logerr("[StateMachine] No task data")
            self.set_state(FlightState.ERROR)
            return
        
        takeoff_alt = self.flight_controller.takeoff_alt
        cruise_height = task_data['cruise_height']
        waypoint_list = task_data['waypoint_list']
        rtl_waypoint_list = task_data['rtl_waypoint_list']
        rtl_type = task_data['rtl_type']
        home_gps = task_data['home_gps']
        
        # ============================================================================
        # 从dest起飞
        # ============================================================================
        if not self.check_flight_mode():
            return
        
        if not self.flight_controller.takeoff(takeoff_alt):
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.ERROR,
                "Return takeoff failed",
                task_data['order_id']
            )
            self.set_state(FlightState.ERROR)
            return
        
        # ============================================================================
        # 升高到巡航高度
        # ============================================================================
        if cruise_height and cruise_height > takeoff_alt:
            rospy.loginfo("[StateMachine] Climbing to cruise height for return")
            
            if not self.check_flight_mode():
                return
            
            if not self.flight_controller.goto_altitude(cruise_height):
                self.mqtt_client.publish_status(
                    task_data['task_id'],
                    FlightState.ERROR,
                    "Failed to reach cruise height for return",
                    task_data['order_id']
                )
                self.set_state(FlightState.ERROR)
                return
        
        # ============================================================================
        # 飞返程waypoints（根据rtl_type决定航线）
        # ============================================================================
        if rtl_type == "reverse":
            return_waypoints = list(reversed(waypoint_list)) if waypoint_list else []
        else:
            return_waypoints = rtl_waypoint_list
        
        if return_waypoints:
            rospy.loginfo("[StateMachine] Flying %d return waypoints", len(return_waypoints))
            
            if not self.check_flight_mode():
                return
            
            if not self.flight_controller.execute_waypoints(return_waypoints, home_gps):
                self.mqtt_client.publish_status(
                    task_data['task_id'],
                    FlightState.ERROR,
                    "Return waypoints failed",
                    task_data['order_id']
                )
                self.set_state(FlightState.ERROR)
                return
        
        # ============================================================================
        # 飞往home（商家起点）
        # ============================================================================
        home_point = {'lat': home_gps[0], 'lng': home_gps[1], 'alt': cruise_height}
        rospy.loginfo("[StateMachine] Flying to home")
        
        if not self.check_flight_mode():
            return
        
        if not self.flight_controller.execute_waypoints([home_point], home_gps):
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.ERROR,
                "Failed to reach home",
                task_data['order_id']
            )
            self.set_state(FlightState.ERROR)
            return
        
        # ============================================================================
        # 设置返程标志，表示到达home而非dest
        # ============================================================================
        self._is_return_phase = True
        
        self.mqtt_client.publish_status(
            task_data['task_id'],
            FlightState.ARRIVED,
            "Arrived at home, hovering for land command",
            task_data['order_id']
        )
        self.set_state(FlightState.ARRIVED)
    
    def _handle_rtl_landing(self):
        """
        返航降落状态处理
        
        阶段说明：
        - 在home降落
        - 完成后进入COMPLETED状态
        """
        task_data = self.get_task_data()
        
        if not self.check_flight_mode():
            return
        
        if not self.flight_controller.land():
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.ERROR,
                "Home landing failed",
                task_data['order_id']
            )
            self.set_state(FlightState.ERROR)
        else:
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.COMPLETED,
                "Task completed",
                task_data['order_id']
            )
            self.set_state(FlightState.COMPLETED)
    
    def _handle_completed(self):
        """
        完成状态处理
        
        任务完成，清理状态，回到IDLE
        """
        rospy.loginfo("[StateMachine] Task completed")
        self._is_return_phase = False
        self.set_task_data(None)
        self.clear_requests()
        self.flight_controller.set_abort(False)
        self.set_state(FlightState.IDLE)
    
    def _handle_cancelled(self):
        """
        取消状态处理
        
        任务取消，清理状态，回到IDLE
        """
        task_data = self.get_task_data()
        if task_data:
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.CANCELLED,
                "Task cancelled",
                task_data.get('order_id', '')
            )
        
        rospy.loginfo("[StateMachine] Task cancelled")
        self._is_return_phase = False
        self.set_task_data(None)
        self.clear_requests()
        self.flight_controller.set_abort(False)
        self.set_state(FlightState.IDLE)
    
    def _handle_failed(self):
        """
        失败状态处理
        
        任务失败，清理状态，回到IDLE
        """
        rospy.loginfo("[StateMachine] Task failed")
        self._is_return_phase = False
        self.set_task_data(None)
        self.clear_requests()
        self.flight_controller.set_abort(False)
        self.set_state(FlightState.IDLE)
    
    def _handle_error(self):
        """
        错误状态处理
        
        任务错误，清理状态，回到IDLE
        """
        rospy.loginfo("[StateMachine] Task error")
        self._is_return_phase = False
        self.set_task_data(None)
        self.clear_requests()
        self.flight_controller.set_abort(False)
        self.set_state(FlightState.IDLE)
    
    def _handle_aborted(self):
        """
        紧急中断状态处理
        
        安全机制：
        - 检测到飞行模式变化（非OFFBOARD）时自动进入
        - 停止所有设定点发布
        - 清理状态，回到IDLE
        """
        task_data = self.get_task_data()
        if task_data:
            self.mqtt_client.publish_status(
                task_data['task_id'],
                FlightState.ABORTED,
                "Task aborted - flight mode changed",
                task_data.get('order_id', '')
            )
        
        rospy.logwarn("[StateMachine] Task aborted, returning to IDLE")
        self._is_return_phase = False
        self.set_state(FlightState.ABORTED)
        self.set_task_data(None)
        self.clear_requests()
        self.flight_controller.set_abort(False)
        
        rospy.sleep(1.0)
        self.set_state(FlightState.IDLE)


# =============================================================================
# MQTT客户端类
# =============================================================================

class MqttClient:
    """
    MQTT客户端类
    
    负责：
    - 连接MQTT Broker
    - 订阅任务和控制指令主题
    - 发布遥测和状态信息
    """
    
    def __init__(self, node):
        """
        初始化MQTT客户端
        
        Args:
            node: 父节点对象（CoffeeProjNode），用于回调处理
        """
        self.node = node

        #调试参数
        self.debug_log_telemetry = rospy.get_param('~debug_log_telemetry', False)
        self.broker_ip = rospy.get_param('~broker_ip', '8.136.222.199')
        self.broker_port = rospy.get_param('~broker_port', 1883)
        self.keepalive = rospy.get_param('~keepalive', 60)
        self.mqtt_user = rospy.get_param('~mqtt_user', 'drone_001')
        self.mqtt_pw = rospy.get_param('~mqtt_pw', 'drone_001')
        self.drone_id = rospy.get_param('~drone_id', 'drone_001')
        
        self.TOPIC_TASK = f"drone/{self.drone_id}/task"
        self.TOPIC_COMMAND = f"drone/{self.drone_id}/command"
        self.TOPIC_TELEMETRY = f"drone/{self.drone_id}/telemetry"
        self.TOPIC_STATUS = f"drone/{self.drone_id}/status"
        
        self._mqtt_connected = False
        self._msg_id = 1
        
        self._mqtt = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'coffee_{self.drone_id}',
            protocol=mqtt.MQTTv311,
            transport="websockets"
        )
        self._mqtt.username_pw_set(self.mqtt_user, self.mqtt_pw)
        self._mqtt.tls_set(cert_reqs=ssl.CERT_REQUIRED)
        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect
        self._mqtt.on_message = self._on_message
        
        self.connect()
    
    def connect(self):
        """
        连接MQTT Broker并启动后台线程
        
        启动后自动重连
        """
        try:
            self._mqtt.connect(self.broker_ip, self.broker_port, keepalive=self.keepalive)
            self._mqtt.loop_start()
            rospy.loginfo("[MQTT] Connecting to %s:%d", self.broker_ip, self.broker_port)
        except Exception as exc:
            rospy.logerr("[MQTT] Connect error: %s", exc)
    
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        """
        MQTT连接成功回调
        
        连接成功后订阅任务和控制主题
        """
        rc_value = rc.value if hasattr(rc, 'value') else rc
        if rc_value == 0 or rc_value == 'Success':
            self._mqtt_connected = True
            client.subscribe(self.TOPIC_TASK, qos=1)
            client.subscribe(self.TOPIC_COMMAND, qos=1)
            rospy.loginfo("[MQTT] Connected. Subscribed to [%s, %s]", 
                          self.TOPIC_TASK, self.TOPIC_COMMAND)
        else:
            rospy.logerr("[MQTT] Connection refused (rc=%s)", rc)
    
    def _on_disconnect(self, client, userdata, rc, properties=None, reason_code=None):
        """
        MQTT断连回调
        
        自动重连
        """
        self._mqtt_connected = False
        rospy.logwarn("[MQTT] Disconnected (rc=%s), reconnecting...", rc)
    
    def _on_message(self, client, userdata, msg):
        """
        MQTT消息接收回调
        
        解析JSON并转发给父节点处理
        """
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            if topic == self.TOPIC_TASK:
                self.node.handle_task(payload)
            elif topic == self.TOPIC_COMMAND:
                self.node.handle_command(payload)
            else:
                rospy.logwarn("[MQTT] Unknown topic: %s", topic)
                
        except json.JSONDecodeError as exc:
            rospy.logerr("[MQTT] JSON parse error: %s", exc)
        except Exception as exc:
            rospy.logerr("[MQTT] Message handler error: %s", exc)
    
    def is_connected(self):
        """
        检查MQTT是否已连接
        
        Returns:
            True: 已连接, False: 未连接
        """
        return self._mqtt_connected
    
    def _next_msg_id(self):
        """
        生成下一个消息ID（自增）
        
        Returns:
            新的消息ID
        """
        self._msg_id += 1
        return self._msg_id
    
    def publish_telemetry(self, data):
        """
        发布遥测数据到MQTT
        
        Args:
            data: 包含遥测数据的字典，包含以下字段：
                - taskId: 任务ID
                - status: 状态字符串
                - lat, lng, alt: GPS位置
                - heading: 航向角（度）
                - speed: 速度（m/s）
                - battery: 电池电量百分比
                - gpsStatus: GPS状态
                - flightMode: 飞行模式
        """
        if not self._mqtt_connected:
            return
        if not self.debug_log_telemetry:
            payload = {
                "msgID": self._next_msg_id(),
                "timestamp": now_ms(),
                "source": self.drone_id,
                "droneId": self.drone_id,
                "taskId": data.get('taskId', None),
                "status": data.get('status', 'GROUNDED'),
                "position": {
                    "lat": data.get('lat', 0.0),
                    "lng": data.get('lng', 0.0),
                    "alt": data.get('alt', 0.0),
                    "heading": data.get('heading', 0.0),
                    "speed": data.get('speed', 0.0),
                    "battery": data.get('battery', 0.0),
                    "gpsStatus": data.get('gpsStatus', 'no_fix'),
                    "flightMode": data.get('flightMode', 'manual')
                }
            }
        elif self.debug_log_telemetry:
            payload = {
                "msgID": self._next_msg_id(),
                "timestamp": now_ms(),
                "source": self.drone_id,
                "droneId": self.drone_id,
                "taskId": data.get('taskId', None),
                "status": data.get('status', 'GROUNDED'),
                "position": {
                    "lat":40.154442,
                    "lng":116.263554,
                    "alt": 10.0,
                    "heading": data.get('heading', 0.0),
                    "speed": data.get('speed', 0.0),
                    "battery": data.get('battery', 0.0),
                    "gpsStatus": data.get('gpsStatus', 'no_fix'),
                    "flightMode": data.get('flightMode', 'manual')
                }
            }
        # if self.debug_log_telemetry:
        rospy.loginfo("[MQTT] Publishing telemetry: taskId=%s status=%s pos=(%.6f, %.6f, %.1f) battery=%.1f%% mode=%s",
                    payload['taskId'], payload['status'], 
                    payload['position']['lat'], payload['position']['lng'], payload['position']['alt'],
                    payload['position']['battery'], payload['position']['flightMode'])
            
        self._mqtt.publish(self.TOPIC_TELEMETRY, json.dumps(payload), qos=1)
    
    def publish_status(self, task_id, task_state, message="", order_id=None):
        """
        发布任务状态到MQTT
        
        Args:
            task_id: 任务ID
            task_state: 任务状态（FlightState枚举值）
            message: 状态消息说明
            order_id: 订单ID（可选）
        """
        if not self._mqtt_connected:
            return
        
        payload = {
            "msgId": self._next_msg_id(),
            "timestamp": now_ms(),
            "source": self.drone_id,
            "droneId": self.drone_id,
            "orderId": order_id if order_id else "",
            "taskId": task_id,
            "type": "task_status",
            "taskState": task_state,
            "message": message
        }
        
        self._mqtt.publish(self.TOPIC_STATUS, json.dumps(payload), qos=1)
        rospy.loginfo("[MQTT] Status published: state=%d msg=%s orderId=%s", task_state, message, order_id)
    
    def shutdown(self):
        """
        关闭MQTT连接
        
        停止后台线程并断开连接
        """
        self._mqtt.loop_stop()
        self._mqtt.disconnect()


# =============================================================================
# 飞行控制类
# =============================================================================

class FlightController:
    """
    飞行控制器
    
    负责：
    - MAVROS服务调用（解锁、模式切换、降落）
    - 位置控制（起飞、航点飞行、悬停）
    - 紧急中断控制
    
    注意：飞行状态由 TaskStateMachine 管理，本类不维护状态
    """
    
    def __init__(self, node):
        """
        初始化飞行控制器
        
        Args:
            node: 父节点对象（CoffeeProjNode）
        """
        self.node = node
        
        self.takeoff_alt = rospy.get_param('~takeoff_alt', 5.0)
        self.unload_timeS = rospy.get_param('~unload_timeS', 30.0)
        self.wp_tolerance = rospy.get_param('~waypoint_tolerance', 1.5)
        self.pre_land_alt = rospy.get_param('~pre_land_alt', 3.0)
        self.ground_threshold = rospy.get_param('~ground_threshold', 0.25)
        self.rth_alt = rospy.get_param('~rth_alt', 10.0)
        self.phase_timeout = rospy.get_param('~phase_timeout', 120.0)
        self.setpoint_rate = rospy.get_param('~setpoint_rate', 20.0)

        
        self.debug_setPosition_gps = rospy.get_param('~debug_setPosition_gps', True)
        
        self._mav_state = State()
        self._local_pose = PoseStamped()
        self._home_pos = None
        
        self._abort = False
        
        # 延迟加载EGM96 geoid模型（用于椭球高到AMSL的转换）
        # 避免全局初始化阻塞节点启动
        self._geoid = None
        
        self._sp_pub = rospy.Publisher(
            rospy.get_param('~mavros_setpoint_pos', '/mavros/setpoint_position/local'),
            PoseStamped, queue_size=10
        )
        self._spgps_pub = rospy.Publisher(
            rospy.get_param('~mavros_setpoint_pos_global', '/mavros/setpoint_position/global'),
            GeoPoseStamped, queue_size=10
        )
        
        rospy.Subscriber(
            rospy.get_param('~mavros_state', '/mavros/state'),
            State, self._cb_state, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param('~mavros_local_pose', '/mavros/local_position/pose'),
            PoseStamped, self._cb_local_pose, queue_size=1
        )
        rospy.Subscriber(
            rospy.get_param('~mavros_home_pos', '/mavros/home_position/home'),
            HomePosition, self._cb_home_pos, queue_size=1
        )
        
        self._wait_mavros_services()
    
    def _wait_mavros_services(self):
        """
        等待MAVROS服务可用
        
        包括解锁服务和模式切换服务
        """
        rospy.loginfo("[Flight] Waiting for MAVROS services...")
        try:
            rospy.wait_for_service(rospy.get_param('~mavros_arming_srv', '/mavros/cmd/arming'), timeout=15.0)
            rospy.wait_for_service(rospy.get_param('~mavros_set_mode_srv', '/mavros/set_mode'), timeout=15.0)
            rospy.loginfo("[Flight] MAVROS services available.")
        except rospy.ROSException as e:
            rospy.logwarn("[Flight] MAVROS services not available yet.")
        
        self._arm_client = rospy.ServiceProxy(
            rospy.get_param('~mavros_arming_srv', '/mavros/cmd/arming'), CommandBool
        )
        self._mode_client = rospy.ServiceProxy(
            rospy.get_param('~mavros_set_mode_srv', '/mavros/set_mode'), SetMode
        )
    
    def _get_geoid(self):
        """
        延迟加载EGM96 geoid模型
        
        Returns:
            GeoidPGM实例
            
        Note: 延迟加载避免全局初始化阻塞节点启动
        """
        if self._geoid is None:
            rospy.loginfo("[Flight] Loading EGM96 geoid model...")
            self._geoid = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
            rospy.loginfo("[Flight] EGM96 geoid model loaded successfully")
        return self._geoid
    
    def _cb_state(self, msg):
        """MAVROS状态回调"""
        self._mav_state = msg
    
    def _cb_local_pose(self, msg):
        """本地位置回调"""
        self._local_pose = msg
    
    def _cb_home_pos(self, msg):
        """Home位置回调"""
        self._home_pos = msg
        
        lat = msg.geo.latitude
        lon = msg.geo.longitude
        ellipsoid_alt = msg.geo.altitude
        
        # 延迟加载geoid模型并计算geoid高度
        geoid_height = self._get_geoid().height(lat, lon)
        amsl_alt = ellipsoid_alt - geoid_height
        
        # rospy.loginfo("[Flight] HOME UPDATE: lat=%.6f, lon=%.6f, ellipsoid=%.1f m, geoid=%.1f m -> AMSL=%.1f m",
        #               lat, lon, ellipsoid_alt, geoid_height, amsl_alt)
    
    def get_local_pose(self):
        """
        获取当前位置（ENU坐标）
        
        Returns:
            PoseStamped消息
        """
        return self._local_pose
    
    def get_home_gps(self):
        """
        获取Home点的GPS坐标
        
        Returns:
            (lat, lon, alt) 元组，或None如果未设置
            
        Note: MAVROS geo.altitude是WGS84椭球高，需转换为AMSL
        """
        if self._home_pos:
            lat = self._home_pos.geo.latitude
            lon = self._home_pos.geo.longitude
            ellipsoid_alt = self._home_pos.geo.altitude
            
            # 延迟加载geoid模型并计算AMSL高度
            geoid_height = self._get_geoid().height(lat, lon)
            amsl_alt = ellipsoid_alt - geoid_height
            
            return (lat, lon, amsl_alt)
        return None

    def agl_to_amsl(self, agl_height):
        """
        Convert AGL (Above Ground Level) height to AMSL (Above Mean Sea Level) altitude
        
        Args:
            agl_height: Height relative to ground (meters)
        
        Returns:
            AMSL altitude (meters), or None if no home position
        """
        home_gps = self.get_home_gps()
        if home_gps:
            lat, lon, home_amsl = home_gps
            
            # 延迟加载geoid模型并计算geoid高度
            geoid_height = self._get_geoid().height(lat, lon)
            ellipsoid_alt = home_amsl + geoid_height
            
            amsl = home_amsl + agl_height
            rospy.loginfo("[Flight] AGL=%.1f m + home(AMSL=%.1f m) -> AMSL=%.1f m (ellipsoid=%.1f m)",
                          agl_height, home_amsl, amsl, amsl + geoid_height)
            return amsl
        rospy.logerr("[Flight] Cannot convert AGL to AMSL: no home position!")
        return None

    
    def is_armed(self):
        """
        检查是否已解锁
        
        Returns:
            True: 已解锁, False: 未解锁
        """
        return self._mav_state.armed
    
    def is_offboard(self):
        """
        检查是否在OFFBOARD模式
        
        Returns:
            True: OFFBOARD模式, False: 其他模式
        """
        return self._mav_state.mode == "OFFBOARD"
    
    def make_setpoint(self, x, y, z):
        """
        创建位置设定点消息
        
        Args:
            x: 东向位置（米）
            y: 北向位置（米）
            z: 向上位置（米）
        
        Returns:
            PoseStamped消息
        """
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.w = 1.0
        return sp
    def make_setpointgps(self, lat, lon, alt):
        """
        创建位置设定点消息
        对于/mavros/setpoint_position/global主题，altitude是相对于海平面的绝对高度(amsl)，而不是相对地面的增量。确保传入的alt参数是正确的绝对高度。
        对于读取/mavros/global_position/global主题，alt是椭球高,而不是相对于海平面的高度。如果需要相对高度，可以通过减去当地的地面高度来计算。
        绝对不要在获取的高度基础上直接加上起飞高度作为设定点的高度
        Args:
                lat: 纬度
                lon: 经度
                alt: 高度
        
        Returns:
            GeoPoseStamped消息
        """
        sp = GeoPoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.latitude = lat
        sp.pose.position.longitude = lon
        sp.pose.position.altitude = alt 
        sp.pose.orientation.w = 1.0
        return sp
    
    def set_mode(self, mode, retries=5):
        """
        设置飞行模式
        
        Args:
            mode: 模式名称（如'OFFBOARD', 'AUTO.LAND'等）
            retries: 重试次数
        
        Returns:
            True: 成功, False: 失败
        """
        req = SetModeRequest(custom_mode=mode)
        for _ in range(retries):
            try:
                resp = self._mode_client(req)
                if resp.mode_sent:
                    rospy.loginfo("[Flight] Mode set to %s", mode)
                    return True
            except rospy.ServiceException as exc:
                rospy.logwarn("[Flight] set_mode failed: %s", exc)
            rospy.sleep(0.5)
        return False
    
    def arm(self, arm=True, retries=5):
        """
        解锁/上锁电机
        
        Args:
            arm: True解锁, False上锁
            retries: 重试次数
        
        Returns:
            True: 成功, False: 失败
        """
        req = CommandBoolRequest(value=arm)
        for _ in range(retries):
            try:
                resp = self._arm_client(req)
                if resp.success:
                    rospy.loginfo("[Flight] Arm=%s success", arm)
                    return True
            except rospy.ServiceException as exc:
                rospy.logwarn("[Flight] arm failed: %s", exc)
            rospy.sleep(0.5)
        return False
    

    def fly_to_gps(self, lat, lon, agl_height, tol=None, timeout=None):
        """
        Fly to specified GPS coordinates and AGL height
        
        Args:
            lat: Target latitude (degrees)
            lon: Target longitude (degrees)
            agl_height: Height above ground (meters)
            tol: Tolerance (meters), default wp_tolerance
            timeout: Timeout (seconds), default phase_timeout
        
        Returns:
            True: reached target, False: timeout or interrupted
        """
        if tol is None:
            tol = self.wp_tolerance
        if timeout is None:
            timeout = self.phase_timeout
        
        amsl_alt = self.agl_to_amsl(agl_height)
        if amsl_alt is None:
            rospy.logerr("[Flight] fly_to_gps failed: no home position")
            return False
        
        rospy.loginfo("[Flight] GPS FLY TO: lat=%.6f, lon=%.6f, agl=%.1f m (amsl=%.1f m)", 
                      lat, lon, agl_height, amsl_alt)
        
        sp = self.make_setpointgps(lat, lon, amsl_alt)
        rate = rospy.Rate(self.setpoint_rate)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        
        while not rospy.is_shutdown() and not self._abort:
            sp.header.stamp = rospy.Time.now()
            try:
                self._spgps_pub.publish(sp)
            except rospy.ROSException as e:
                rospy.logerr("[Flight] fly_to_gps publish failed: %s", e)
                return False
            
            if self._abort:
                rospy.logwarn("[Flight] fly_to_gps aborted")
                return False
            
            # Check if reached target
            current_lat = self.node._global_pos.latitude
            current_lon = self.node._global_pos.longitude
            current_agl = self.node._relative_alt
            
            # Calculate horizontal distance using haversine approximation
            dlat = math.radians(current_lat - lat)
            dlon = math.radians(current_lon - lon)
            avg_lat = math.radians((current_lat + lat) / 2.0)
            horiz_dist = _EARTH_R * math.sqrt(dlat**2 + (dlon * math.cos(avg_lat))**2)
            vert_dist = abs(current_agl - agl_height)
            
            # Log progress periodically
            # if int((rospy.Time.now() - deadline + rospy.Duration(timeout)).to_sec()) % 5 == 0:
            #     rospy.loginfo("[Flight] GPS progress: horiz=%.2f m, vert=%.2f m (target: horiz<%.1f, vert<%.1f)",
            #                   horiz_dist, vert_dist, tol, tol)
            
            if horiz_dist < tol and vert_dist < tol:
                # rospy.loginfo("[Flight] Reached GPS target (horiz=%.2f m, vert=%.2f m)", horiz_dist, vert_dist)
                return True
            
            if rospy.Time.now() > deadline:
                rospy.logerr("[Flight] fly_to_gps timed out: horiz=%.2f m, vert=%.2f m (tol=%.1f)",
                             horiz_dist, vert_dist, tol)
                return False
            
            rate.sleep()
        
        return False

    def fly_to(self, x, y, z, tol=None, timeout=None):
        """
        飞行到指定位置
        
        Args:
            x: 目标东向位置（米）
            y: 目标北向位置（米）
            z: 目标高度（米，相对地面AGL）
            tol: 到达容差（米），默认使用wp_tolerance
            timeout: 超时时间（秒），默认使用phase_timeout
        
        Returns:
            True: 到达目标, False: 超时或中断
        """
        if self.debug_setPosition_gps:
            # GPS mode: convert ENU to GPS and call fly_to_gps
            home_gps = self.get_home_gps()
            if home_gps:
                lat, lon = enu_to_gps_xy(x, y, home_gps[0], home_gps[1])
                return self.fly_to_gps(lat, lon, z, tol, timeout)
            rospy.logerr("[Flight] fly_to GPS mode failed: no home position")
            return False
        
        if tol is None:
            tol = self.wp_tolerance
        if timeout is None:
            timeout = self.phase_timeout
        
        rate = rospy.Rate(self.setpoint_rate)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        sp = self.make_setpoint(x, y, z)
        
        while not rospy.is_shutdown() and not self._abort:
            sp.header.stamp = rospy.Time.now()
            try:
                self._sp_pub.publish(sp)
            except rospy.ROSException as e:
                rospy.loginfo("[Flight] Topic closed, stopping fly_to")
                return False
            
            if self._abort:
                rospy.logwarn("[Flight] fly_to aborted")
                return False
            
            if distance_3d(self._local_pose, x, y, z) < tol:
                return True
            
            if rospy.Time.now() > deadline:
                rospy.logerr("[Flight] fly_to (%.1f,%.1f,%.1f) timed out", x, y, z)
                return False
            
            rate.sleep()
        return False
    
    def hover(self):
        """
        悬停在当前位置
        
        发布当前位置作为设定点，保持悬停
        """
        p = self._local_pose.pose.position
        sp = self.make_setpoint(p.x, p.y, p.z)
        sp.header.stamp = rospy.Time.now()
        try:
            self._sp_pub.publish(sp)
        except rospy.ROSException as e:
            pass
    

    def takeoff_gps(self, agl_height=None):
        """
        Takeoff to specified AGL height at current GPS position
        
        Args:
            agl_height: Height above ground (meters), default takeoff_alt
        
        Returns:
            True: success, False: failed
        """
        if agl_height is None:
            agl_height = self.takeoff_alt
        
        home_gps = self.get_home_gps()
        if home_gps is None:
            rospy.logerr("[Flight] takeoff_gps failed: no home position")
            return False
        
        lat, lon, _ = home_gps
        amsl_alt = self.agl_to_amsl(agl_height)
        
        rospy.loginfo("[Flight] GPS TAKEOFF to %.1f m (amsl=%.1f m)", agl_height, amsl_alt)
        
        sp = self.make_setpointgps(lat, lon, amsl_alt)
        rate = rospy.Rate(self.setpoint_rate)
        
        # Pre-publish setpoints before OFFBOARD (MAVROS requirement: ~100 setpoints)
        rospy.loginfo("[Flight] Publishing GPS setpoints before OFFBOARD...")
        for _ in range(100):
            if rospy.is_shutdown():
                return False
            sp.header.stamp = rospy.Time.now()
            try:
                self._spgps_pub.publish(sp)
            except rospy.ROSException as e:
                rospy.logerr("[Flight] takeoff_gps pre-publish failed: %s", e)
                return False
            rate.sleep()
        
        # Switch to OFFBOARD while continuously sending setpoints
        rospy.loginfo("[Flight] Switching to OFFBOARD mode...")
        mode_success = False
        for attempt in range(5):
            sp.header.stamp = rospy.Time.now()
            self._spgps_pub.publish(sp)
            
            req = SetModeRequest(custom_mode='OFFBOARD')
            try:
                resp = self._mode_client(req)
                if resp.mode_sent:
                    rospy.loginfo("[Flight] Mode set to OFFBOARD")
                    mode_success = True
                    break
            except rospy.ServiceException as exc:
                rospy.logwarn("[Flight] set_mode attempt %d failed: %s", attempt, exc)
            
            # Keep publishing while waiting
            for _ in range(10):
                sp.header.stamp = rospy.Time.now()
                self._spgps_pub.publish(sp)
                rate.sleep()
        
        if not mode_success:
            rospy.logerr("[Flight] Failed to set OFFBOARD mode")
            return False
        
        # Arm while continuously sending setpoints
        rospy.loginfo("[Flight] Arming...")
        arm_success = False
        for attempt in range(5):
            sp.header.stamp = rospy.Time.now()
            self._spgps_pub.publish(sp)
            
            req = CommandBoolRequest(value=True)
            try:
                resp = self._arm_client(req)
                if resp.success:
                    rospy.loginfo("[Flight] Armed successfully")
                    arm_success = True
                    break
            except rospy.ServiceException as exc:
                rospy.logwarn("[Flight] arm attempt %d failed: %s", attempt, exc)
            
            # Keep publishing while waiting
            for _ in range(10):
                sp.header.stamp = rospy.Time.now()
                self._spgps_pub.publish(sp)
                rate.sleep()
        
        if not arm_success:
            rospy.logerr("[Flight] Failed to arm")
            return False
        
        rospy.loginfo("[Flight] Armed, climbing to target altitude...")
        
        # Wait until reached target altitude (keep publishing setpoints)
        deadline = rospy.Time.now() + rospy.Duration(self.phase_timeout)
        while not rospy.is_shutdown() and not self._abort:
            sp.header.stamp = rospy.Time.now()
            try:
                self._spgps_pub.publish(sp)
            except rospy.ROSException as e:
                rospy.logerr("[Flight] takeoff_gps wait publish failed: %s", e)
                return False
            
            current_agl = self.node._relative_alt
            if current_agl >= agl_height - 0.5:
                rospy.loginfo("[Flight] GPS TAKEOFF complete at %.1f m", current_agl)
                return True
            
            if rospy.Time.now() > deadline:
                rospy.logerr("[Flight] takeoff_gps timed out at %.1f m", current_agl)
                return False
            
            rate.sleep()
        
        return False

    def takeoff(self, alt=None):
        """
        起飞到指定高度
        
        流程：发送设定点 → 切换OFFBOARD → 解锁 → 等待到达目标高度
        
        Args:
            alt: 起飞高度（米），相对地面增量，默认使用takeoff_alt参数
        
        Returns:
            True: 起飞成功, False: 失败
        """
        if self.debug_setPosition_gps:
            return self.takeoff_gps(alt)
        

        if alt is None:
            alt = self.takeoff_alt
        
        home_x = self._local_pose.pose.position.x
        home_y = self._local_pose.pose.position.y
        home_z = self._local_pose.pose.position.z
        target_z = home_z + alt
        
        rospy.loginfo("[Flight] TAKEOFF from %.1f m to %.1f m (height=%.1f m)", 
                      home_z, target_z, alt)
        
        rate = rospy.Rate(self.setpoint_rate)
        
        sp = self.make_setpoint(home_x, home_y, target_z)
        
        for _ in range(50):
            if rospy.is_shutdown():
                return False
            sp.header.stamp = rospy.Time.now()
            try:
                self._sp_pub.publish(sp)
            except rospy.ROSException as e:
                rospy.loginfo("[Flight] Topic closed during takeoff")
                return False
            rate.sleep()
        
        if not self.set_mode('OFFBOARD'):
            return False
        
        rospy.sleep(0.5)
        
        if not self.arm(True):
            return False
        
        if self.fly_to(home_x, home_y, target_z, tol=0.5):
            return True
        return False
    
    def land(self):
        """
        执行降落
        
        调用MAVROS降落服务，等待落地并上锁
        
        Returns:
            True: 降落成功, False: 超时或中断
        """
        rospy.loginfo("[Flight] LANDING")
        
        try:
            land_client = rospy.ServiceProxy(
                rospy.get_param('~mavros_land_srv', '/mavros/cmd/land'), CommandTOL
            )
            resp = land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            if not resp.success:
                rospy.logwarn("[Flight] Land command failed")
        except rospy.ServiceException as exc:
            rospy.logerr("[Flight] Land service error: %s", exc)
        
        deadline = rospy.Time.now() + rospy.Duration(60.0)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown() and not self._abort:
            if not self._mav_state.armed:
                rospy.loginfo("[Flight] Disarmed - landing complete")
                return True
            if rospy.Time.now() > deadline:
                rospy.logerr("[Flight] Landing timeout")
                return False
            rate.sleep()
        return False
    

    def goto_altitude_gps(self, target_agl, tol=0.5, timeout=None):
        """
        Climb/descend to specified AGL height while holding current GPS position
        
        Args:
            target_agl: Target height above ground (meters)
            tol: Tolerance (meters)
            timeout: Timeout (seconds)
        
        Returns:
            True: reached altitude, False: timeout or interrupted
        """
        if timeout is None:
            timeout = self.phase_timeout
        
        current_lat = self.node._global_pos.latitude
        current_lon = self.node._global_pos.longitude
        
        amsl_alt = self.agl_to_amsl(target_agl)
        if amsl_alt is None:
            rospy.logerr("[Flight] goto_altitude_gps failed: no home position")
            return False
        
        rospy.loginfo("[Flight] GPS GOTO ALTITUDE to %.1f m (amsl=%.1f m)", target_agl, amsl_alt)
        
        sp = self.make_setpointgps(current_lat, current_lon, amsl_alt)
        rate = rospy.Rate(self.setpoint_rate)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        
        while not rospy.is_shutdown() and not self._abort:
            # Update position (use current GPS position)
            current_lat = self.node._global_pos.latitude
            current_lon = self.node._global_pos.longitude
            
            sp.pose.position.latitude = current_lat
            sp.pose.position.longitude = current_lon
            sp.header.stamp = rospy.Time.now()
            
            try:
                self._spgps_pub.publish(sp)
            except rospy.ROSException as e:
                rospy.logerr("[Flight] goto_altitude_gps publish failed: %s", e)
                return False
            
            current_agl = self.node._relative_alt
            if abs(current_agl - target_agl) < tol:
                rospy.loginfo("[Flight] Reached GPS altitude %.1f m", target_agl)
                return True
            
            if rospy.Time.now() > deadline:
                rospy.logerr("[Flight] goto_altitude_gps timed out at %.1f m", current_agl)
                return False
            
            rate.sleep()
        
        return False

    def goto_altitude(self, target_alt, tol=0.5, timeout=None):
        """
        升高/降低到目标高度（使用位置控制而非takeoff）
        
        Args:
            target_alt: 目标高度（米，相对地面AGL）
            tol: 到达容差（米）
            timeout: 超时时间（秒）
        
        Returns:
            True: 到达目标高度, False: 超时或中断
        """
        if self.debug_setPosition_gps:
            return self.goto_altitude_gps(target_alt, tol, timeout)
        
        if timeout is None:
            timeout = self.phase_timeout
        
        rospy.loginfo("[Flight] GOTO altitude to %.1f m", target_alt)
        
        p = self._local_pose.pose.position
        current_alt = p.z
        
        rospy.loginfo("[Flight] Current: %.1f m → Target: %.1f m", current_alt, target_alt)
        
        rate = rospy.Rate(self.setpoint_rate)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        
        while not rospy.is_shutdown() and not self._abort:
            sp = self.make_setpoint(p.x, p.y, target_alt)
            sp.header.stamp = rospy.Time.now()
            try:
                self._sp_pub.publish(sp)
            except rospy.ROSException as e:
                rospy.loginfo("[Flight] Topic closed during goto_altitude")
                return False
            
            current_z = self._local_pose.pose.position.z
            if abs(current_z - target_alt) < tol:
                rospy.loginfo("[Flight] Reached target altitude %.1f m", target_alt)
                return True
            
            if rospy.Time.now() > deadline:
                rospy.logerr("[Flight] goto_altitude timed out")
                return False
            
            rate.sleep()
        
        return False
    
    def set_abort(self, abort=True):
        """
        设置紧急中断标志
        
        Args:
            abort: True设置中断, False清除中断
        """
        self._abort = abort
        if abort:
            rospy.logwarn("[Flight] ABORT flag set!")
    
    def is_aborted(self):
        """
        检查是否已中断
        
        Returns:
            True: 已中断, False: 未中断
        """
        return self._abort
    

    def execute_waypoints_gps(self, waypoints):
        """
        Execute waypoint flight using GPS coordinates
        
        Args:
            waypoints: List of waypoints, each with lat, lng, alt (AGL height in meters)
        
        Returns:
            True: all waypoints reached, False: failed
        """
        for i, wp in enumerate(waypoints):
            if self._abort:
                rospy.logwarn("[Flight] execute_waypoints_gps aborted at waypoint %d", i)
                return False
            
            lat = wp.get('lat', 0)
            lon = wp.get('lng', 0)
            agl = wp.get('alt', 0)
            
            rospy.loginfo("[Flight] GPS waypoint %d: lat=%.6f, lon=%.6f, agl=%.1f m", i+1, lat, lon, agl)
            
            if not self.fly_to_gps(lat, lon, agl):
                rospy.logerr("[Flight] Failed to reach GPS waypoint %d", i+1)
                return False
        
        return True

    def execute_waypoints(self, waypoints, home_gps=None):
        """
        执行航点列表
        
        Args:
            waypoints: 航点列表，每个航点包含 lat, lng, alt (AGL高度)
            home_gps: Home点GPS坐标 (lat, lon, alt)，用于坐标转换（仅local模式使用）
        
        Returns:
            True: 全部航点完成, False: 失败或中断
        """
        if self.debug_setPosition_gps:
            # GPS模式：直接使用原始GPS坐标，不做任何转换
            return self.execute_waypoints_gps(waypoints)
        
        # local模式：需要做GPS→ENU转换
        if home_gps is None:
            home_gps = self.get_home_gps()
        
        if home_gps is None:
            rospy.logerr("[Flight] No home GPS position!")
            return False
        
        home_lat, home_lon, _ = home_gps
        
        for i, wp in enumerate(waypoints):
            if self._abort:
                rospy.logwarn("[Flight] execute_waypoints aborted at waypoint %d", i)
                return False
            
            lat = wp.get('lat', 0)
            lon = wp.get('lng', 0)
            alt = wp.get('alt', 0)
            

            x, y = gps_to_enu_xy(lat, lon, home_lat, home_lon)
            z = alt
            
            rospy.loginfo("[Flight] Flying to waypoint %d: (%.2f, %.2f, %.2f)", i+1, x, y, z)
            
            if not self.fly_to(x, y, z):
                rospy.logerr("[Flight] Failed to reach waypoint %d", i+1)
                return False
        
        return True
    
    def rth(self, rtl_type="reverse", waypoint_list=None, rtl_waypoint_list=None, home_gps=None):
        """
        返航（Return to Home）
        
        Args:
            rtl_type: 返航类型
                - "reverse": 沿原路返回（反转waypoint_list）
                - "new": 使用新的rtl_waypoint_list
            waypoint_list: 原航点列表（用于reverse模式）
            rtl_waypoint_list: RTL专用航点列表（用于new模式）
            home_gps: Home点GPS坐标
        
        Returns:
            True: 返航成功, False: 失败
        """
        rospy.loginfo("[Flight] RTH - type: %s", rtl_type)
        
        if rtl_type == "reverse":
            waypoints = list(reversed(waypoint_list or []))
        else:
            waypoints = rtl_waypoint_list or []
        
        if waypoints:
            if not self.execute_waypoints(waypoints, home_gps):
                return False
        
        return self.land()

    def return_to_home(self, land=True):
        """
        返航到飞机的home点（ENU原点）
        
        用于测试目的：先确保在空中，再飞到坐标原点
        
        Args:
            land: True降落在home点, False仅悬停
        
        Returns:
            True: 返航成功, False: 失败
        """
        rospy.loginfo("[Flight] Returning to drone home (ENU origin) - ONLY FOR SIM TESTING")
        
        current_z = self._local_pose.pose.position.z
        
        # 如果在地面或很低，先起飞
        if current_z < self.ground_threshold + 0.5:
            rospy.loginfo("[Flight] Aircraft on ground, taking off first...")
            if not self.takeoff(self.takeoff_alt):
                rospy.logerr("[Flight] Failed to takeoff for RTH")
                return False
            current_z = self._local_pose.pose.position.z
        
        # 飞到原点（保持当前高度）
        if not self.fly_to(0.0, 0.0, current_z):
            rospy.logerr("[Flight] Failed to reach home position")
            return False
        
        rospy.loginfo("[Flight] Reached home position")
        
        if land:
            return self.land()
        
        return True
# =============================================================================
# 主节点类
# =============================================================================

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
        
        self.mqtt_client = MqttClient(self)
        self.flight_controller = FlightController(self)
        
        # ============================================================================
        # 初始化并启动状态机
        # ============================================================================
        self.state_machine = TaskStateMachine(self, self.flight_controller, self.mqtt_client)
        self.state_machine.start()
        
        rospy.Timer(
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
            # 调试：确认定时器触发（使用loginfo确保能看到）
            rospy.loginfo("[CoffeeProj] Telemetry timer triggered")
            
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
        
        # ============================================================================
        # 设置任务数据并进入LOADED状态
        # ============================================================================
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
            # 注意：rtl指令仅用于仿真测试，实飞环境不应使用.他会从5米高度直接飞回原点并降落。
            # 此处使用返航指令替代卸货完成时的起飞指令
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
        
        # 中断当前任务
        self.state_machine.request_abort()
        
        # 执行返航测试
        t = threading.Thread(target=self.flight_controller.return_to_home, daemon=True)
        t.start()
    
    def run(self):
        """
        启动节点主循环
        
        阻塞运行直到节点关闭
        """
        rospy.spin()
        
        self.state_machine.stop()
        self.mqtt_client.shutdown()


if __name__ == '__main__':
    node = CoffeeProjNode()
    node.run()