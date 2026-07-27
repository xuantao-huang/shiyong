#!/usr/bin/env python3
"""
rc_multipoint_control.py — RC-based multi-waypoint navigation control node

RC Channel 8 (index 7, momentary switch) as the only trigger button.

Config (config/config.yaml):
  auto_planning: 0/1  — 1=auto start navigation after takeoff
  auto_landing:  0/1  — 1=auto land after last waypoint
  (old mode=0 ≡ both 0; mode=1 ≡ both 1)

Waypoints are read from config/config.yaml (relative to this script).
"""

import rospy
import numpy as np
import yaml
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn
from quadrotor_msgs.msg import TakeoffLand, Px4ctrlState

# ── RC thresholds ──
CH8_DOWN = 1990       # momentary button pressed

# ── Step enum ──
STEP_TAKEOFF  = 0
STEP_NAVIGATE = 1
STEP_LAND     = 2


class RCMultipointControl:
    def __init__(self):
        rospy.init_node('rc_multipoint_control')

        # ── config YAML ──
        yaml_rel = rospy.get_param("~yaml_path", "config/config.yaml")
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.yaml_path = os.path.join(base_dir, yaml_rel)
        self._cfg = self._load_config()
        cfg = self._cfg or {}

        # ── parameters (ROS param overrides YAML) ──
        self.auto_planning = rospy.get_param("~auto_planning", cfg.get("auto_planning", 0))
        self.auto_landing  = rospy.get_param("~auto_landing",  cfg.get("auto_landing", 0))
        self.arrival_threshold = rospy.get_param("~arrival_threshold", 1.0)
        self.auto_land_delay   = rospy.get_param("~auto_land_delay", 3.0)
        self.waypoint_timeout   = rospy.get_param("~waypoint_timeout", 120.0)

        # ── waypoints ──
        self.waypoints = []
        self.load_waypoints()

        # ── state ──
        self.current_wp_idx = 0
        self.navigating = False
        self.rc_step = STEP_TAKEOFF
        self.px4_state = Px4ctrlState.MANUAL_CTRL
        self.odom_pos = np.zeros(3)
        self.rc8_prev = False
        self.last_press = rospy.Time(0)
        self.wp_start_time = None                # per-waypoint timeout timer

        # ── readiness tracking ──
        self._rc_ok = False
        self._odom_ok = False
        self._state_ok = False
        self._ready_shown = False

        # ── subscribers ──
        rospy.Subscriber("/mavros/rc/in",   RCIn,          self.rc_cb)
        rospy.Subscriber("/px4ctrl/state",  Px4ctrlState,  self.px4_state_cb)
        rospy.Subscriber("/ekf/ekf_odom",   Odometry,      self.odom_cb)

        # ── publishers ──
        self.takeoff_land_pub = rospy.Publisher(
            "/px4ctrl/takeoff_land", TakeoffLand, queue_size=1)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)

        # ── main loop @ 10Hz ──
        self.timer = rospy.Timer(rospy.Duration(0.1), self.main_loop)

        rospy.loginfo("[rc] Node started, auto_planning=%d, auto_landing=%d, waypoints=%d",
                      self.auto_planning, self.auto_landing, len(self.waypoints))
        rospy.loginfo("[rc] Checking system readiness...")
        rospy.spin()

    # ─────────────────────────────────────────────────
    #  YAML loading
    # ─────────────────────────────────────────────────
    def _load_config(self):
        """Load config YAML, return dict or None on failure."""
        if not os.path.exists(self.yaml_path):
            rospy.logerr("[rc] Config file not found: %s", self.yaml_path)
            return None
        with open(self.yaml_path, 'r') as f:
            return yaml.safe_load(f)

    def load_waypoints(self):
        """Load waypoints from pre-loaded config dict."""
        if self._cfg is None:
            return
        if 'points' not in self._cfg:
            rospy.logerr("[rc] Invalid YAML format. Expected 'points:' section.")
            return

        raw = self._cfg['points']
        if raw is None:
            rospy.logwarn("[rc] 'points:' section is empty.")
            return

        for i, pt in enumerate(raw):
            if not isinstance(pt, (list, tuple)) or len(pt) < 3:
                rospy.logerr("[rc] Waypoint %d format error, need at least [x, y, z], got: %s", i+1, pt)
                continue
            wp = list(pt)
            # pad missing field: time defaults to 0
            while len(wp) < 4:
                wp.append(0)
            self.waypoints.append(wp)
            rospy.loginfo("[rc]   wp %d: [%.1f, %.1f, %.1f]%s",
                          i+1, wp[0], wp[1], wp[2],
                          f" wait={wp[3]:.1f}s" if wp[3] > 0 else "")

        if not self.waypoints:
            rospy.logerr("[rc] No valid waypoints loaded!")
        else:
            rospy.loginfo("[rc] Loaded %d waypoints from %s", len(self.waypoints), self.yaml_path)

    # ─────────────────────────────────────────────────
    #  Readiness check
    # ─────────────────────────────────────────────────
    def check_readiness(self):
        if self._ready_shown:
            return
        if self._rc_ok and self._odom_ok and self._state_ok:
            self._ready_shown = True
            ap = "ON" if self.auto_planning else "OFF"
            al = "ON" if self.auto_landing else "OFF"
            rospy.loginfo("=" * 50)
            rospy.loginfo("  ALL SYSTEMS READY")
            rospy.loginfo("  Auto-planning: %s  |  Auto-landing: %s", ap, al)
            rospy.loginfo("  Waypoints: %d  |  CH8: trigger button", len(self.waypoints))
            rospy.loginfo("  CH8 press 1 -> TAKEOFF")
            if not self.auto_planning:
                rospy.loginfo("  CH8 press 2 -> NAVIGATE waypoints")
            else:
                rospy.loginfo("  Navigation will start automatically after takeoff")
            if not self.auto_landing:
                rospy.loginfo("  CH8 press %d -> LAND", 3 if not self.auto_planning else 2)
            else:
                rospy.loginfo("  Landing will trigger automatically after last waypoint")
            rospy.loginfo("=" * 50)
        else:
            missing = []
            if not self._rc_ok:    missing.append("RC")
            if not self._odom_ok:  missing.append("odom")
            if not self._state_ok: missing.append("px4ctrl")
            rospy.loginfo_throttle(3, "[rc] Waiting for: %s ...", ", ".join(missing))

    # ─────────────────────────────────────────────────
    #  Main loop  @ 10Hz
    # ─────────────────────────────────────────────────
    def main_loop(self, event):
        self.check_readiness()
        if not self._ready_shown:
            return

        # auto-planning: after takeoff completes, auto-start navigation
        if (self.auto_planning and self.rc_step == STEP_NAVIGATE
                and not self.navigating
                and self.px4_state == Px4ctrlState.AUTO_HOVER
                and self.waypoints):
            self.current_wp_idx = 0
            self.navigating = True
            self.wp_start_time = rospy.Time.now()
            self._publish_waypoint(0)
            rospy.loginfo("[rc] Auto-planning: NAVIGATE (%d waypoints)", len(self.waypoints))

        # monitor navigation progress
        if self.navigating:
            self._monitor_navigation()

        # auto-landing: after last waypoint, auto land
        if (self.auto_landing and self.rc_step == STEP_LAND
                and not self.navigating
                and self.px4_state == Px4ctrlState.AUTO_HOVER):
            rospy.sleep(self.auto_land_delay)
            self._send_land()
            self.rc_step = STEP_TAKEOFF
            self.wp_start_time = None
            rospy.loginfo("[rc] Auto-landing: LAND, sequence complete")

    # ─────────────────────────────────────────────────
    #  RC callback
    # ─────────────────────────────────────────────────
    def rc_cb(self, msg):
        self._rc_ok = True

        # CH8 rising edge (momentary button)
        ch8 = msg.channels[7] if len(msg.channels) > 7 else 0
        pressed = (ch8 >= CH8_DOWN)
        if pressed and not self.rc8_prev:
            now = rospy.Time.now()
            if (now - self.last_press).to_sec() > 1.0:
                self._on_trigger()
                self.last_press = now
        self.rc8_prev = pressed

    def px4_state_cb(self, msg):
        self._state_ok = True
        self.px4_state = msg.state

    def odom_cb(self, msg):
        self._odom_ok = True
        self.odom_pos[0] = msg.pose.pose.position.x
        self.odom_pos[1] = msg.pose.pose.position.y
        self.odom_pos[2] = msg.pose.pose.position.z

    # ─────────────────────────────────────────────────
    #  Trigger handler (unified, always step-based)
    # ─────────────────────────────────────────────────
    def _on_trigger(self):
        if self.rc_step == STEP_TAKEOFF:
            self._send_takeoff()
            rospy.loginfo("[rc] >>> TAKEOFF")
            self.rc_step = STEP_NAVIGATE

        elif self.rc_step == STEP_NAVIGATE:
            if self.px4_state == Px4ctrlState.AUTO_HOVER and self.waypoints and not self.navigating:
                self.current_wp_idx = 0
                self.navigating = True
                self.wp_start_time = rospy.Time.now()
                self._publish_waypoint(0)
                rospy.loginfo("[rc] >>> NAVIGATE (%d waypoints)", len(self.waypoints))

        elif self.rc_step == STEP_LAND:
            if self.px4_state == Px4ctrlState.AUTO_HOVER:
                self._send_land()
                rospy.loginfo("[rc] >>> LAND")
                self.rc_step = STEP_TAKEOFF
                self.wp_start_time = None

    # ─────────────────────────────────────────────────
    #  Navigation monitor  (@ 10Hz)
    # ─────────────────────────────────────────────────
    def _monitor_navigation(self):
        # per-waypoint timeout
        if self.wp_start_time is not None:
            elapsed = (rospy.Time.now() - self.wp_start_time).to_sec()
            if elapsed > self.waypoint_timeout:
                rospy.logerr("[rc] Waypoint %d timeout (%.0fs), skipping",
                             self.current_wp_idx + 1, elapsed)
                self._advance_waypoint()
                return

        wp = self.waypoints[self.current_wp_idx]
        dist = np.linalg.norm(self.odom_pos - np.array(wp[:3]))
        if dist < self.arrival_threshold:
            if wp[3] > 0:
                rospy.loginfo("[rc] Waiting %.1fs at waypoint %d", wp[3], self.current_wp_idx + 1)
                rospy.sleep(wp[3])
            self._advance_waypoint()

    def _advance_waypoint(self):
        self.current_wp_idx += 1
        if self.current_wp_idx < len(self.waypoints):
            self._publish_waypoint(self.current_wp_idx)
            self.wp_start_time = rospy.Time.now()
        else:
            self.navigating = False
            self.rc_step = STEP_LAND
            rospy.loginfo("[rc] All waypoints done. %s.",
                          "Auto-landing..." if self.auto_landing else "Hovering. CH8 to land.")

    # ─────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────
    def _send_takeoff(self):
        tl = TakeoffLand()
        tl.takeoff_land_cmd = TakeoffLand.TAKEOFF
        self.takeoff_land_pub.publish(tl)

    def _send_land(self):
        tl = TakeoffLand()
        tl.takeoff_land_cmd = TakeoffLand.LAND
        self.takeoff_land_pub.publish(tl)

    def _publish_waypoint(self, idx):
        wp = self.waypoints[idx]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = wp[0]
        goal.pose.position.y = wp[1]
        goal.pose.position.z = wp[2]
        self.goal_pub.publish(goal)
        rospy.loginfo("[rc] Waypoint %d/%d: (%.1f, %.1f, %.1f)",
                      idx+1, len(self.waypoints), wp[0], wp[1], wp[2])


if __name__ == "__main__":
    RCMultipointControl()
