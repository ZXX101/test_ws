#!/usr/bin/env python3
"""
utils.py
=========
工具函数模块

提供坐标转换、距离计算、时间戳等基础工具函数，
以及飞行任务状态枚举。不依赖 ROS 节点或其他业务类。
"""

import math
import time
from enum import IntEnum
from typing import Any


# =============================================================================
# 常量
# =============================================================================

_EARTH_R = 6_371_000.0


# =============================================================================
# 工具函数
# =============================================================================


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


def distance_3d(pose: Any, x_b: float, y_b: float, z_b: float) -> float:
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


def now_ms() -> int:
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