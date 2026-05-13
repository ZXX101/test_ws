#!/usr/bin/env python3
"""
state_machine.py
================
任务状态机模块

负责：
- 状态管理和转换
- 状态执行逻辑
- 安全检查（飞行模式监控）
- 中断处理
"""

from __future__ import annotations

import os
import sys
import threading

import rospy

if TYPE_CHECKING:
    from coffee_proj_node import CoffeeProjNode
    from flight_controller import FlightController
    from mqtt_client import MqttClient

_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from utils import FlightState
from typing import TYPE_CHECKING


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

    def __init__(
        self,
        node: CoffeeProjNode,
        flight_controller: FlightController,
        mqtt_client: MqttClient
    ):
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
        if self._abort_requested:
            self._handle_aborted()
            return

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

                    rospy.sleep(0.5)

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