#!/usr/bin/env python3
"""
mavros_test_node.py
MAVROS 单项测试工具
- SetOffboard: 切换到 OFFBOARD 模式
- Takeoff: 起飞到指定高度
- Land: 降落
"""

import rospy
import threading
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest


class MavrosTestController:
    def __init__(self):
        rospy.init_node('mavros_test_node', anonymous=False)
        
        self.takeoff_alt = rospy.get_param('~takeoff_alt', 5.0)
        self.setpoint_rate = rospy.get_param('~setpoint_rate', 20.0)
        self.debug_setPosition_gps = rospy.get_param('~debug_setPosition_gps', True)
        
        self._mav_state = State()
        self._local_pose = PoseStamped()
        self._global_pos = NavSatFix()
        self._relative_alt = 0.0
        self._battery = BatteryState()
        self._home_pos = None
        
        rospy.Subscriber('/mavros/state', State, self._cb_state, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._cb_local_pose, queue_size=1)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self._cb_global_pos, queue_size=1)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self._cb_rel_alt, queue_size=1)
        rospy.Subscriber('/mavros/battery', BatteryState, self._cb_battery, queue_size=1)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self._cb_home_pos, queue_size=1)
        
        self._sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self._spgps_pub = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
        
        self._wait_mavros_services()
        
        rospy.loginfo("[TestTool] MAVROS Test Controller initialized")
    
    def _wait_mavros_services(self):
        rospy.loginfo("[TestTool] Waiting for MAVROS services...")
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=15.0)
            rospy.wait_for_service('/mavros/set_mode', timeout=15.0)
            rospy.loginfo("[TestTool] MAVROS services available")
        except rospy.ROSException:
            rospy.logwarn("[TestTool] MAVROS services not available")
        
        self._arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self._mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    def _cb_state(self, msg):
        self._mav_state = msg
        rospy.loginfo_throttle(5.0, "[Telemetry] Mode: %s, Armed: %s", msg.mode, msg.armed)
    
    def _cb_local_pose(self, msg):
        self._local_pose = msg
        rospy.loginfo_throttle(5.0, "[Telemetry] Local: x=%.2f y=%.2f z=%.2f", 
                               msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    
    def _cb_global_pos(self, msg):
        self._global_pos = msg
        rospy.loginfo_throttle(5.0, "[Telemetry] GPS: lat=%.6f lon=%.6f alt=%.1f", 
                               msg.latitude, msg.longitude, msg.altitude)
    
    def _cb_rel_alt(self, msg):
        self._relative_alt = msg.data
        rospy.loginfo_throttle(5.0, "[Telemetry] RelAlt: %.2f m", msg.data)
    
    def _cb_battery(self, msg):
        self._battery = msg
        rospy.loginfo_throttle(5.0, "[Telemetry] Battery: %.1f%%", msg.percentage * 100.0)
    
    def _cb_home_pos(self, msg):
        self._home_pos = msg
        rospy.loginfo("[Telemetry] Home: lat=%.6f lon=%.6f", 
                      msg.geo.latitude, msg.geo.longitude)
    
    def make_setpoint(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.w = 1.0
        return sp
    
    def make_setpointgps(self, lat, lon, alt):
        sp = GeoPoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        sp.pose.position.latitude = lat
        sp.pose.position.longitude = lon
        sp.pose.position.altitude = alt
        sp.pose.orientation.w = 1.0
        return sp
    
    def set_mode(self, mode, retries=5):
        req = SetModeRequest(custom_mode=mode)
        for i in range(retries):
            try:
                resp = self._mode_client(req)
                if resp.mode_sent:
                    rospy.loginfo("[TestTool] Mode set to %s (attempt %d)", mode, i+1)
                    return True
            except rospy.ServiceException as exc:
                rospy.logwarn("[TestTool] set_mode failed: %s", exc)
            rospy.sleep(0.5)
        return False
    
    def arm(self, arm=True, retries=5):
        req = CommandBoolRequest(value=arm)
        for i in range(retries):
            try:
                resp = self._arm_client(req)
                if resp.success:
                    rospy.loginfo("[TestTool] Arm=%s success (attempt %d)", arm, i+1)
                    return True
            except rospy.ServiceException as exc:
                rospy.logwarn("[TestTool] arm failed: %s", exc)
            rospy.sleep(0.5)
        return False
    
    def is_armed(self):
        return self._mav_state.armed
    
    def is_offboard(self):
        return self._mav_state.mode == "OFFBOARD"
    
    def set_offboard(self):
        rospy.loginfo("[TestTool] Setting OFFBOARD mode...")
        
        if self.is_offboard():
            rospy.loginfo("[TestTool] Already in OFFBOARD mode")
            return True
        
        rate = rospy.Rate(self.setpoint_rate)
        
        if self.debug_setPosition_gps:
            if self._home_pos is None:
                rospy.logerr("[TestTool] No home position, cannot use GPS mode")
                return False
            
            lat = self._home_pos.geo.latitude
            lon = self._home_pos.geo.longitude
            alt = self._home_pos.geo.altitude + self.takeoff_alt
            
            sp = self.make_setpointgps(lat, lon, alt)
            for _ in range(100):
                sp.header.stamp = rospy.Time.now()
                self._spgps_pub.publish(sp)
                rate.sleep()
        else:
            p = self._local_pose.pose.position
            sp = self.make_setpoint(p.x, p.y, p.z + self.takeoff_alt)
            for _ in range(50):
                sp.header.stamp = rospy.Time.now()
                self._sp_pub.publish(sp)
                rate.sleep()
        
        if not self.set_mode('OFFBOARD'):
            rospy.logerr("[TestTool] Failed to set OFFBOARD mode")
            return False
        
        rospy.sleep(0.5)
        rospy.loginfo("[TestTool] OFFBOARD mode set successfully")
        return True
    
    def takeoff(self, alt=None):
        if alt is None:
            alt = self.takeoff_alt
        
        rospy.loginfo("[TestTool] Takeoff to %.1f m", alt)
        
        if not self.is_offboard():
            if not self.set_offboard():
                return False
        
        if not self.is_armed():
            rospy.loginfo("[TestTool] Arming...")
            if not self.arm(True):
                rospy.logerr("[TestTool] Failed to arm")
                return False
        
        rate = rospy.Rate(self.setpoint_rate)
        
        if self.debug_setPosition_gps:
            if self._home_pos is None:
                rospy.logerr("[TestTool] No home position")
                return False
            
            lat = self._home_pos.geo.latitude
            lon = self._home_pos.geo.longitude
            amsl_alt = self._home_pos.geo.altitude + alt
            
            sp = self.make_setpointgps(lat, lon, amsl_alt)
            
            rospy.loginfo("[TestTool] Climbing to %.1f m...", alt)
            while not rospy.is_shutdown():
                sp.header.stamp = rospy.Time.now()
                self._spgps_pub.publish(sp)
                
                if self._relative_alt >= alt - 0.5:
                    rospy.loginfo("[TestTool] Reached altitude %.1f m", self._relative_alt)
                    return True
                
                rate.sleep()
        else:
            p = self._local_pose.pose.position
            target_z = p.z + alt
            
            sp = self.make_setpoint(p.x, p.y, target_z)
            
            rospy.loginfo("[TestTool] Climbing to %.1f m...", alt)
            while not rospy.is_shutdown():
                sp.header.stamp = rospy.Time.now()
                self._sp_pub.publish(sp)
                
                current_z = self._local_pose.pose.position.z
                if current_z >= target_z - 0.5:
                    rospy.loginfo("[TestTool] Reached altitude %.1f m", current_z)
                    return True
                
                rate.sleep()
        
        return False
    
    def land(self):
        rospy.loginfo("[TestTool] Landing...")
        
        try:
            land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            resp = land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            if not resp.success:
                rospy.logwarn("[TestTool] Land command failed")
        except rospy.ServiceException as exc:
            rospy.logerr("[TestTool] Land service error: %s", exc)
        
        rospy.loginfo("[TestTool] Waiting for landing to complete...")
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if not self._mav_state.armed:
                rospy.loginfo("[TestTool] Disarmed - landing complete")
                return True
            rate.sleep()
        
        return False
    
    def print_telemetry(self):
        rospy.loginfo("\n[Telemetry Summary]")
        rospy.loginfo("  Flight Mode: %s", self._mav_state.mode)
        rospy.loginfo("  Armed: %s", self._mav_state.armed)
        rospy.loginfo("  Local Position: x=%.2f y=%.2f z=%.2f", 
                      self._local_pose.pose.position.x,
                      self._local_pose.pose.position.y,
                      self._local_pose.pose.position.z)
        rospy.loginfo("  GPS: lat=%.6f lon=%.6f", 
                      self._global_pos.latitude, self._global_pos.longitude)
        rospy.loginfo("  Relative Alt: %.2f m", self._relative_alt)
        rospy.loginfo("  Battery: %.1f%%", self._battery.percentage * 100.0)


def show_menu():
    print("\n" + "="*50)
    print(" MAVROS 单项测试工具")
    print("="*50)
    print("  1. Set OFFBOARD mode")
    print("  2. Takeoff to altitude")
    print("  3. Land")
    print("  4. Show telemetry")
    print("  0. Exit")
    print("="*50)


def main():
    controller = MavrosTestController()
    
    rospy.loginfo("[TestTool] Waiting for telemetry data...")
    rospy.sleep(2.0)
    
    while not rospy.is_shutdown():
        show_menu()
        
        try:
            choice = input("Select test (0-4): ").strip()
        except EOFError:
            break
        
        if choice == '0':
            rospy.loginfo("[TestTool] Exiting...")
            break
        elif choice == '1':
            rospy.loginfo("[TestTool] Testing: Set OFFBOARD")
            t = threading.Thread(target=controller.set_offboard, daemon=True)
            t.start()
            t.join(timeout=10.0)
        # elif choice == '2':
        #     try:
        #         alt_input = input("Enter altitude (m, default 5.0): ").strip()
        #         alt = float(alt_input) if alt_input else 5.0
        #     except ValueError:
        #         alt = 5.0
            
        #     rospy.loginfo("[TestTool] Testing: Takeoff to %.1f m", alt)
        #     t = threading.Thread(target=controller.takeoff, args=(alt,), daemon=True)
        #     t.start()
        #     t.join(timeout=30.0)
        # elif choice == '3':
        #     rospy.loginfo("[TestTool] Testing: Land")
        #     t = threading.Thread(target=controller.land, daemon=True)
        #     t.start()
        #     t.join(timeout=60.0)
        # elif choice == '4':
        #     controller.print_telemetry()
        else:
            rospy.logwarn("[TestTool] Invalid choice: %s", choice)
        
        rospy.sleep(0.5)


if __name__ == '__main__':
    main()