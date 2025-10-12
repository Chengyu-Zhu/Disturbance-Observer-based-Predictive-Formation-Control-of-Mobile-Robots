#!/usr/bin/env python3

import sys
import os
import numpy as np

# Try to import ROS modules; allow non-ROS environments to import this file for testing.
try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32
    ROS_AVAILABLE = True
except Exception:
    rospy = None
    Twist = None
    Odometry = None
    Float32 = None
    ROS_AVAILABLE = False

from Dual_MPC import DualMPC

# Global state from odometry callback
_g_state = np.array([0.0, 0.0, 0.0])
_g_tao1 = np.array([0.0])
_g_tao2 = np.array([0.0])
_g_tao3 = 0.0


def odom_callback(msg):
    """Update global pose from Odometry message."""
    global _g_state
    try:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
    except Exception:
        # fallback if message fields differ
        x = 0.0
        y = 0.0
        z = 0.0
    _g_state = np.array([x, y, z])

def robot1_callback(msg):
    global _g_tao1
    try:
        # expect Float32
        tao1 = float(msg.data)
    except Exception:
        # 防止除零
        tao1 = 1e-12
    _g_tao1 = tao1

def robot2_callback(msg):
    global _g_tao2
    try:
        tao2 = float(msg.data)
    except Exception:
        # 防止除零
        tao2 = 1e-13
    _g_tao2 = tao2


class RosMPCNode:
    def __init__(self, rate_hz=10):
        rospy.init_node('robot3', anonymous=True)
        self.rate = rospy.Rate(rate_hz)
        self.pub = rospy.Publisher('/motor_control', Twist, queue_size=1)
        rospy.Subscriber('/odomuwb', Odometry, odom_callback, queue_size=1)
        # subscribe to tao values published as Float32 on /robot1_tao and /robot2_tao
        rospy.Subscriber('/robot1_tao', Float32, robot1_callback, queue_size=1)
        rospy.Subscriber('/robot2_tao', Float32, robot2_callback, queue_size=1)
        # publisher for tao3
        self.tao3_pub = rospy.Publisher('/robot3_tao', Float32, queue_size=1)

        # create MPC
        self.mpc = DualMPC()
        self.mpc.sys_init()
        # ensure alpha initialized large for feasibility (matches dualmode_mpc behavior)
        try:
            # prefer existing attribute but set a large default
            self.mpc.alpha = float(getattr(self.mpc, 'alpha', 100000.0))
        except Exception:
            self.mpc.alpha = 100000.0
        self.tao = 0.0

        # initial reference and constraints
        X_o_d = np.array([0.0, 0.0, 0.0])
        self.mpc.error_state = np.zeros(3)
        self.mpc.update_ref(self.tao)
        self.mpc.constraint_init(X_o_d[0], X_o_d[1], X_o_d[2])

        # internal logging buffers
        self.state_log = []
        self.input_log = []
        self.ref_log = []

    def step(self):
        global _g_state
        global _g_tao1
        global _g_tao2
        # compute control sequence using our SQP/QCQP solver
        c_seq = self.mpc.solve_control(float(_g_tao1), float(_g_tao2))

        if hasattr(self.mpc, 'Cal_alpha'):
            self.mpc.Cal_alpha(c_seq)

        # compute error input and update internal error state
        u_err = self.mpc.Cal_error_input(c_seq)
        self.mpc.error_state = self.mpc.Cal_error_state(c_seq)

        # desired/true input
        desire_input = np.array([self.mpc.v if hasattr(self.mpc, 'v') else 0.0,
                                  self.mpc.w if hasattr(self.mpc, 'w') else 0.0])

        # publish Twist message
        vel_msg = Twist()
        vel_msg.linear.x = float(desire_input[0] + u_err[0])
        vel_msg.angular.z = float(desire_input[1] + u_err[1])
        self.pub.publish(vel_msg)

        self.tao += 0.25
        # compute scalar tao3 (using inner product with Z)
        try:
            vec = np.array([desire_input[0] + float(u_err[0]), desire_input[1] + float(u_err[1])])
            tao3 = float(self.tao + self.mpc.k_i * float(self.mpc.Z @ vec))
        except Exception:
            tao3 = float(self.tao)
        if ROS_AVAILABLE and Float32 is not None:
            self.tao3_pub.publish(Float32(data=tao3))

        X_o_d = np.array([0.01 * self.tao - 1, 2 * np.sin(0.01 * self.tao - 1) + 3, 0.0])
        self.mpc.update_ref(self.tao)
        self.mpc.constraint_init(X_o_d[0], X_o_d[1], X_o_d[2])

        # logging
        self.state_log.append((_g_state.copy()))
        self.input_log.append((vel_msg.linear.x, vel_msg.angular.z))
        self.ref_log.append(X_o_d.copy())

    def run(self):
        rospy.loginfo('robot3 started')
        while not rospy.is_shutdown():
            t0 = rospy.Time.now()
            self.step()
            # ensure loop rate
            self.rate.sleep()


def main():
    node = RosMPCNode(rate_hz=10)
    try:
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('robot3 terminated')
        out_dir = os.path.join(os.path.dirname(__file__), 'output')
        os.makedirs(out_dir, exist_ok=True)
        np.savetxt(os.path.join(out_dir, 'state_log.txt'), np.array(node.state_log))
        np.savetxt(os.path.join(out_dir, 'input_log.txt'), np.array(node.input_log))
        np.savetxt(os.path.join(out_dir, 'ref_log.txt'), np.array(node.ref_log))


if __name__ == '__main__':
    main()
