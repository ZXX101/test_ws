#!/usr/bin/env python3
"""
zeroone_rtk.py
==============
RTK数据转发模块

负责：
- 从MQTT /px4/rtkdata话题接收RTK数据
- 将RTK数据完整转发到串口（用于RTK基站）

数据特性：
- 频率：1HZ
- 每条数据最大800B，通常600B左右
- 需要确保完整接收和完整发送，不可拆分
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import TYPE_CHECKING

import rospy
import serial

if TYPE_CHECKING:
    from coffee_proj_node import CoffeeProjNode

_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)


class ZerooneRtk:
    """
    RTK数据转发器

    从MQTT订阅RTK数据并转发到串口

    使用方式：
    - 由主节点创建实例
    - 主节点在MQTT连接成功后调用register_mqtt_subscription()
    - 主节点在MQTT消息回调中调用handle_rtk_data()
    """

    TOPIC_RTKDATA = "/px4/rtkdata"

    def __init__(self, node: CoffeeProjNode):
        """
        初始化RTK转发器

        Args:
            node: 主节点对象（CoffeeProjNode）
        """
        self.node = node

        # 串口参数
        self.serial_port = rospy.get_param('~rtk_serial_port', '/dev/ttyUSBACM0')
        self.serial_baudrate = rospy.get_param('~rtk_serial_baudrate', 115200)
        self.serial_timeout = rospy.get_param('~rtk_serial_timeout', 1.0)

        # 串口对象
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._serial_connected = False

        # 统计信息
        self._msg_count = 0
        self._error_count = 0
        self._last_msg_time = 0.0

        # 启用标志
        self._enabled = rospy.get_param('~rtk_enabled', True)

        if self._enabled:
            self._connect_serial()
            rospy.loginfo("[ZerooneRtk] Initialized, serial_port=%s, baudrate=%d",
                          self.serial_port, self.serial_baudrate)
        else:
            rospy.loginfo("[ZerooneRtk] Disabled by parameter")

    def _connect_serial(self):
        """
        连接串口

        尝试打开串口设备，失败时记录错误
        """
        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout
            )
            self._serial_connected = True
            rospy.loginfo("[ZerooneRtk] Serial connected: %s @ %d",
                          self.serial_port, self.serial_baudrate)
        except serial.SerialException as e:
            self._serial_connected = False
            self._serial = None
            rospy.logwarn("[ZerooneRtk] Failed to connect serial: %s", e)
        except Exception as e:
            self._serial_connected = False
            self._serial = None
            rospy.logerr("[ZerooneRtk] Unexpected error connecting serial: %s", e)

    def _reconnect_serial(self):
        """
        重连串口

        关闭现有连接并重新连接
        """
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.close()
                except Exception:
                    pass

            self._serial = None
            self._serial_connected = False

            # 等待一段时间后重连
            time.sleep(0.5)
            self._connect_serial()

    def register_mqtt_subscription(self, mqtt_client):
        """
        注册MQTT订阅

        在MQTT连接成功后调用，订阅RTK数据话题

        Args:
            mqtt_client: MqttClient实例
        """
        if not self._enabled:
            return

        # 直接使用底层的paho mqtt客户端订阅
        mqtt_client._mqtt.subscribe(self.TOPIC_RTKDATA, qos=0)
        rospy.loginfo("[ZerooneRtk] Subscribed to MQTT topic: %s", self.TOPIC_RTKDATA)

    def handle_rtk_data(self, payload: bytes):
        """
        处理RTK数据

        由主节点的MQTT消息回调调用，接收原始字节数据并转发到串口

        Args:
            payload: RTK数据的原始字节，确保完整的一条数据

        Note:
            - 数据通常是600B左右，最大800B
            - 每秒1条（1HZ）
            - 必须完整发送，不可拆分
        """
        if not self._enabled:
            return

        if not payload:
            rospy.logwarn("[ZerooneRtk] Received empty payload")
            return

        data_len = len(payload)
        rospy.loginfo("[ZerooneRtk] Received RTK data from MQTT: %d bytes", data_len)

        # 检查串口连接
        if not self._serial_connected or not self._serial:
            rospy.logdebug("[ZerooneRtk] Serial not connected, attempting reconnect")
            self._reconnect_serial()

            if not self._serial_connected:
                self._error_count += 1
                return

        # 发送到串口（完整发送，不拆分）
        try:
            with self._serial_lock:
                if self._serial and self._serial.is_open:
                    # 一次性写入完整数据
                    bytes_written = self._serial.write(payload)
                    self._serial.flush()  # 确保数据发送完成

                    if bytes_written != data_len:
                        rospy.logwarn("[ZerooneRtk] Incomplete write: %d/%d bytes",
                                      bytes_written, data_len)
                        self._error_count += 1
                    else:
                        self._msg_count += 1
                        self._last_msg_time = time.time()
                        rospy.logdebug("[ZerooneRtk] Sent to serial: %d bytes", bytes_written)
                else:
                    rospy.logwarn("[ZerooneRtk] Serial port not open")
                    self._error_count += 1
                    self._serial_connected = False

        except serial.SerialTimeoutException as e:
            rospy.logerr("[ZerooneRtk] Serial write timeout: %s", e)
            self._error_count += 1
            self._serial_connected = False
        except serial.SerialException as e:
            rospy.logerr("[ZerooneRtk] Serial error: %s", e)
            self._error_count += 1
            self._serial_connected = False
            self._reconnect_serial()
        except Exception as e:
            rospy.logerr("[ZerooneRtk] Unexpected error: %s", e)
            self._error_count += 1

    def get_stats(self):
        """
        获取统计信息

        Returns:
            dict: 包含消息计数、错误计数、最后消息时间的字典
        """
        return {
            'msg_count': self._msg_count,
            'error_count': self._error_count,
            'last_msg_time': self._last_msg_time,
            'serial_connected': self._serial_connected,
            'enabled': self._enabled
        }

    def is_serial_connected(self):
        """
        检查串口是否连接

        Returns:
            bool: 串口连接状态
        """
        return self._serial_connected

    def shutdown(self):
        """
        关闭RTK转发器

        关闭串口连接
        """
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.close()
                    rospy.loginfo("[ZerooneRtk] Serial port closed")
                except Exception as e:
                    rospy.logwarn("[ZerooneRtk] Error closing serial: %s", e)

            self._serial = None
            self._serial_connected = False

        rospy.loginfo("[ZerooneRtk] Shutdown complete. Stats: msg=%d, errors=%d",
                      self._msg_count, self._error_count)
