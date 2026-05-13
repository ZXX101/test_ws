#!/usr/bin/env python3
"""
flight_controller.py
====================
飞行控制器模块

负责：
- MAVROS服务调用（解锁、模式切换、降落）
- 位置控制（起飞、航点飞行、悬停）
- 紧急中断控制

注意：飞行状态由 TaskStateMachine 管理，本类不维护状态
"""

from __future__ import annotations

import math
import os
import sys
import rospy
from typing import TYPE_CHECKING

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from pygeodesy.geoids import GeoidPGM

if TYPE_CHECKING:
    from coffee_proj_node import CoffeeProjNode

_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from utils import (
    FlightState,
    now_ms,
    gps_to_enu_xy,
    enu_to_gps_xy,
    yaw_from_quaternion,
    distance_3d,
    _EARTH_R,
)


class FlightController:
    """
    飞行控制器

    负责：
    - MAVROS服务调用（解锁、模式切换、降落）
    - 位置控制（起飞、航点飞行、悬停）
    - 紧急中断控制

    注意：飞行状态由 TaskStateMachine 管理，本类不维护状态
    """

    def __init__(self, node: CoffeeProjNode):
        """
        初始化飞行控制器

        Args:
            node: 父节点对象（CoffeeProjNode）
        """
        self.node: CoffeeProjNode = node

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
        self._gps_origin = GeoPointStamped()
        self._home_pos = None

        self._abort = False

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

        rospy.Subscriber(
            rospy.get_param('~mavros_gps_origin', '/mavros/global_position/gp_origin'),
            GeoPointStamped, self._cb_gps_origin, queue_size=1
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

    def _cb_gps_origin(self, msg):
        """GPS原点回调（同Home位置）"""
        self._gps_origin = msg

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

                转成了ENU坐标
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

            current_lat = self.node._global_pos.latitude
            current_lon = self.node._global_pos.longitude
            current_agl = self.node._relative_alt

            dlat = math.radians(current_lat - lat)
            dlon = math.radians(current_lon - lon)
            avg_lat = math.radians((current_lat + lat) / 2.0)
            horiz_dist = _EARTH_R * math.sqrt(dlat**2 + (dlon * math.cos(avg_lat))**2)
            vert_dist = abs(current_agl - agl_height)

            if horiz_dist < tol and vert_dist < tol:
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

            for _ in range(10):
                sp.header.stamp = rospy.Time.now()
                self._spgps_pub.publish(sp)
                rate.sleep()

        if not mode_success:
            rospy.logerr("[Flight] Failed to set OFFBOARD mode")
            rospy.logerr("[Flight] Failed to set OFFBOARD mode:STATE:,%s", self._mav_state.mode)
            return False

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

            for _ in range(10):
                sp.header.stamp = rospy.Time.now()
                self._spgps_pub.publish(sp)
                rate.sleep()

        if not arm_success:
            rospy.logerr("[Flight] Failed to arm")
            return False

        rospy.loginfo("[Flight] Armed, climbing to target altitude...")

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
            return self.execute_waypoints_gps(waypoints)

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
        返航（Return to Home） 任务的返航阶段

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

        if current_z < self.ground_threshold + 0.5:
            rospy.loginfo("[Flight] Aircraft on ground, taking off first...")
            if not self.takeoff(self.takeoff_alt):
                rospy.logerr("[Flight] Failed to takeoff for RTH")
                return False
            current_z = self._local_pose.pose.position.z

        if not self.fly_to_gps(self._gps_origin.position.latitude, self._gps_origin.position.longitude, current_z):
            rospy.logerr("[Flight] Failed to reach home position")
            return False

        rospy.loginfo("[Flight] Reached home position")

        if land:
            return self.land()

        return True