#!/usr/bin/env python3
"""
mqtt_client.py
==============
MQTT客户端模块

负责：
- 连接MQTT Broker
- 订阅任务和控制指令主题
- 发布遥测和状态信息
"""

from __future__ import annotations

import json
import os
import ssl
import sys
import threading
from typing import TYPE_CHECKING 

import rospy
import paho.mqtt.client as mqtt

if TYPE_CHECKING:
    from coffee_proj_node import CoffeeProjNode

_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from utils import FlightState, now_ms
from typing import TYPE_CHECKING


class MqttClient:
    """
    MQTT客户端类

    负责：
    - 连接MQTT Broker
    - 订阅任务和控制指令主题
    - 发布遥测和状态信息
    """

    def __init__(self, node: CoffeeProjNode):
        """
        初始化MQTT客户端

        Args:
            node: 父节点对象（CoffeeProjNode），用于回调处理
        """
        self.node = node

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
            # 注册RTK数据订阅
            if hasattr(self.node, 'zeroone_rtk') and self.node.zeroone_rtk:
                self.node.zeroone_rtk.register_mqtt_subscription(self)
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
        RTK数据（/px4/rtkdata）为二进制数据，直接转发
        """
        try:
            topic = msg.topic

            # RTK数据话题处理（二进制数据）
            if topic == "/px4/rtkdata":
                if hasattr(self.node, 'zeroone_rtk') and self.node.zeroone_rtk:
                    self.node.zeroone_rtk.handle_rtk_data(msg.payload)
                return

            # 其他话题处理（JSON数据）
            payload = json.loads(msg.payload.decode('utf-8'))

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
                    "lat": 40.154442,
                    "lng": 116.263554,
                    "alt": 10.0,
                    "heading": data.get('heading', 0.0),
                    "speed": data.get('speed', 0.0),
                    "battery": data.get('battery', 0.0),
                    "gpsStatus": data.get('gpsStatus', 'no_fix'),
                    "flightMode": data.get('flightMode', 'manual')
                }
            }
        if self.debug_log_telemetry:
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
        rospy.loginfo("[MQTT] Status published: task_state=%d msg=%s orderId=%s", task_state, message, order_id)

    def shutdown(self):
        """
        关闭MQTT连接

        停止后台线程并断开连接
        """
        self._mqtt.loop_stop()
        self._mqtt.disconnect()