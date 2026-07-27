#!/usr/bin/env python3
"""
goal_trigger.py — RC channel 6 goal relay node

Directly monitors RC channel 6 (gear switch) via /mavros/rc/in.
On LOW->HIGH edge, computes a goal point `forward_dist` meters ahead of the
current drone heading and publishes it to /move_base_simple/goal for YOPO.

Bypasses px4ctrl trigger chain entirely — works regardless of px4ctrl state
or whether YOPO is already executing a trajectory.

Usage:
    python3 goal_trigger.py [--dist 10.0] [--odom /vins/imu_propagate]
"""

import rospy
import argparse
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn
from scipy.spatial.transform import Rotation as R

GEAR_CHANNEL     = 5       # channels[5] = RC channel 6 (zero-indexed)
GEAR_SHIFT_VALUE = 0.75    # matches px4ctrl GEAR_SHIFT_VALUE in input.h


class GoalTrigger:
    def __init__(self, forward_dist, odom_topic):
        self.forward_dist = forward_dist
        self.last_gear    = -1.0   # initialised LOW so first HIGH edge is detected
        self.odom         = None

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber(odom_topic,       Odometry, self._odom_cb)
        rospy.Subscriber("/mavros/rc/in",  RCIn,     self._rc_cb)
        rospy.loginfo(f"[goal_trigger] ready  dist={forward_dist} m  odom={odom_topic}")

    def _odom_cb(self, msg):
        self.odom = msg

    def _rc_cb(self, msg):
        if len(msg.channels) <= GEAR_CHANNEL:
            return
        gear = (msg.channels[GEAR_CHANNEL] - 1000.0) / 1000.0
        if self.last_gear < GEAR_SHIFT_VALUE and gear > GEAR_SHIFT_VALUE:
            self._publish_goal()
        self.last_gear = gear

    def _publish_goal(self):
        if self.odom is None:
            rospy.logwarn("[goal_trigger] no odom received yet, skipping")
            return

        q   = self.odom.pose.pose.orientation
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')[0]

        gx = self.odom.pose.pose.position.x + np.cos(yaw) * self.forward_dist
        gy = self.odom.pose.pose.position.y + np.sin(yaw) * self.forward_dist
        gz = self.odom.pose.pose.position.z   # preserve current flight altitude

        goal = PoseStamped()
        goal.header.stamp    = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz
        goal.pose.orientation = q

        self.goal_pub.publish(goal)
        rospy.loginfo(f"[goal_trigger] goal ({gx:.1f}, {gy:.1f}, {gz:.2f})  yaw={np.degrees(yaw):.1f} deg")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dist", type=float, default=3.0,
                        help="forward distance in meters (default: 3.0)")
    parser.add_argument("--odom", type=str,   default="/vins/imu_propagate",
                        help="odometry topic (default: /vins/imu_propagate)")
    args = parser.parse_args()

    rospy.init_node("goal_trigger", anonymous=False)
    GoalTrigger(args.dist, args.odom)
    rospy.spin()
