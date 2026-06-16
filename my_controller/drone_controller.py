# =============================================================================
# drone_controller.py
#
# Runs a real Crazyflie drone inside Gazebo simulation.
# Listens to the drone position/velocity, computes thrust and rotation needed
# to follow a figure-8 path, and sends those commands back out.
#
# The flow is:
#   1. Receive position + velocity updates from the simulator (odom_callback)
#   2. Decide where the drone should be right now (figure-8 or hover)
#   3. Run the position controller -> thrust + desired tilt angles
#   4. Run the attitude controller -> torques to reach those angles
#   5. Publish the command, red trail marker, and path history
# =============================================================================

import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from cf_control_msgs.msg import ThrustAndTorque


# Convert a quaternion rotation into roll/pitch/yaw angles (radians).
# Quaternions are how the simulator stores orientation internally - this just
# converts them into angles we can actually reason about.
# Formula taken from standard robotics textbooks.
def quat_to_euler(qx, qy, qz, qw):
    phi   = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    theta = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
    psi   = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return phi, theta, psi


# Convert velocity from the drone's own perspective into world coordinates.
# The sensor reports "I am moving 0.3 m/s forward from my view" - this turns
# that into "the drone is moving north at 0.3 m/s."
# Rotation matrix formula taken from aerospace ZYX convention.
def body_to_world_vel(vx_b, vy_b, vz_b, phi, theta, psi):
    R = np.array([
        [np.cos(psi)*np.cos(theta),
         np.cos(psi)*np.sin(theta)*np.sin(phi) - np.sin(psi)*np.cos(phi),
         np.cos(psi)*np.sin(theta)*np.cos(phi) + np.sin(psi)*np.sin(phi)],
        [np.sin(psi)*np.cos(theta),
         np.sin(psi)*np.sin(theta)*np.sin(phi) + np.cos(psi)*np.cos(phi),
         np.sin(psi)*np.sin(theta)*np.cos(phi) - np.cos(psi)*np.sin(phi)],
        [-np.sin(theta), np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)]
    ])
    return R @ np.array([vx_b, vy_b, vz_b])


# =============================================================================
# AttitudeController
#
# Makes the drone tilt to the angles the position controller asks for.
# Simple PD law: correct the angle error, resist spinning too fast.
# =============================================================================
class AttitudeController:
    def __init__(self):
        self.kR = 0.005   # How hard to correct angle error
        self.kX = 0.002   # Damping - resists fast rotation

    def compute(self, current_angles, desired_angles, angular_velocity):
        return -self.kR * (current_angles - desired_angles) - self.kX * angular_velocity


# =============================================================================
# PositionController
#
# Figures out thrust and tilt angles needed to reach the target position.
# Uses a PD law on each axis separately.
#
# For z: thrust = mass * (g - kz * height_error - kz_dot * vel_error)
# For x/y: tilt angle = acceleration_needed / g  (basic drone physics)
# =============================================================================
class PositionController:
    def __init__(self):
        self.kx     = 2.0
        self.kx_dot = 3.0
        self.ky     = 2.0
        self.ky_dot = 3.0
        self.kz     = 5.0
        self.kz_dot = 6.0
        self.mass = 0.036
        self.g    = 9.81
        self.max_angle = np.radians(25)   # Never tilt more than 25 degrees

    def compute(self, state, desired_pos, desired_vel=None, dt=0.01, acc_ff=None):
        x, y, z    = state[0:3]
        vx, vy, vz = state[3:6]
        dx, dy, dz = desired_pos

        if desired_vel is None:
            desired_vel = np.zeros(3)
        dvx, dvy, dvz = desired_vel

        if acc_ff is None:
            acc_ff = np.zeros(3)

        # Thrust needed to hold/reach target altitude.
        u1 = self.mass * (self.g
                          - self.kz     * (z  - dz)
                          - self.kz_dot * (vz - dvz))

        # acc_ff is a feedforward term - pre-compensates for known future motion
        # on a curved path (e.g. centripetal acceleration on the figure-8).
        a_x = -self.kx * (x - dx) - self.kx_dot * (vx - dvx) + acc_ff[0]
        a_y = -self.ky * (y - dy) - self.ky_dot * (vy - dvy) + acc_ff[1]

        # Convert desired horizontal acceleration into a tilt angle.
        theta_des = np.clip( a_x / self.g, -self.max_angle, self.max_angle)
        phi_des   = np.clip(-a_y / self.g, -self.max_angle, self.max_angle)

        return u1, phi_des, theta_des


# =============================================================================
# DroneController (main ROS 2 node)
#
# Ties everything together. Handles takeoff, switches to figure-8 once high
# enough, runs both controllers, and publishes commands + visualisation.
#
# State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
# =============================================================================
class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.att_ctrl = AttitudeController()
        self.pos_ctrl = PositionController()
        self.state = np.zeros(12)

        # Figure-8 parameters.
        self.r           = 0.5    # Loop radius in meters
        self.omega       = 0.25   # Angular speed in rad/s
        self.z_target    = 1.0    # Cruising altitude in meters
        self.z_ramp_rate = 0.5    # Climb speed during takeoff: 0.5 m/s

        self.node_start_time  = time.time()
        self.fig8_start_time  = None
        self.fig8_active      = False
        self.fig8_x0          = 0.0   # XY center of figure-8, locked in at activation
        self.fig8_y0          = 0.0

        self.last_time        = time.time()
        self.last_debug_time  = 0.0

self.cmd_pub  = self.create_publisher(ThrustAndTorque, '/cf_control/control_command', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        self.create_subscription(Odometry, '/crazyflie/odom', self.odom_callback, 10)
        self.get_logger().info('Drone controller started - figure-8 trajectory.')

    def odom_callback(self, msg):
        # Unpack position, orientation, and angular velocity from the message.
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        av = msg.twist.twist.angular

        phi, theta, psi = quat_to_euler(q.x, q.y, q.z, q.w)

        # Linear velocity comes in body frame - convert to world frame.
        v_world = body_to_world_vel(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            phi, theta, psi
        )
        self.state = np.array([
            p.x, p.y, p.z,
            v_world[0], v_world[1], v_world[2],
            phi, theta, psi,
            av.x, av.y, av.z
        ])
        self.control_loop()

    def control_loop(self):
        now = time.time()
        # Clamp dt to avoid blowups if messages arrive in bursts.
        dt  = np.clip(now - self.last_time, 0.001, 0.1)
        self.last_time = now

        elapsed = now - self.node_start_time

        # Slowly ramp altitude during takeoff, then hold steady.
        z_setpoint    = min(self.z_target, elapsed * self.z_ramp_rate)
        desired_z_vel = self.z_ramp_rate if z_setpoint < self.z_target else 0.0

        actual_z = self.state[2]

        # Switch from hover to figure-8 once we clear 85 cm.
        # Lock current XY as the centre of the figure-8.
        if not self.fig8_active and actual_z >= 0.85:
            self.fig8_active     = True
            self.fig8_start_time = now
            self.fig8_x0 = self.state[0]
            self.fig8_y0 = self.state[1]
            self.get_logger().info(
                f'Figure-8 started at z={actual_z:.2f}m  '
                f'center=({self.fig8_x0:.2f}, {self.fig8_y0:.2f})'
            )

        if self.fig8_active:
            t = now - self.fig8_start_time

            # Figure-8 via a Lissajous curve: y runs at 2x the frequency of x.
            # These are standard parametric equations - velocity and acceleration
            # are their analytical time-derivatives (no numerical approximation).
            desired_pos = np.array([
                self.fig8_x0 + self.r       * np.sin(self.omega * t),
                self.fig8_y0 + (self.r / 2) * np.sin(2 * self.omega * t),
                z_setpoint
            ])
            desired_vel = np.array([
                self.r * self.omega * np.cos(self.omega * t),
                self.r * self.omega * np.cos(2 * self.omega * t),
                desired_z_vel
            ])
            acc_ff = np.array([
                -self.r       * self.omega**2 * np.sin(self.omega * t),
                -2 * self.r   * self.omega**2 * np.sin(2 * self.omega * t),
                0.0
            ])
        else:
            # Takeoff: hover at current XY.
            desired_pos = np.array([self.state[0], self.state[1], z_setpoint])
            desired_vel = np.array([0.0, 0.0, desired_z_vel])
            acc_ff      = np.zeros(3)

        u1, phi_des, theta_des = self.pos_ctrl.compute(
            self.state, desired_pos, desired_vel, dt, acc_ff
        )

        # A tilted drone produces less upward lift - correct for this.
        # max(0.5, ...) prevents divide-by-zero if the drone is very tilted.
        phi_actual   = self.state[6]
        theta_actual = self.state[7]
        tilt_factor  = max(0.5, np.cos(phi_actual) * np.cos(theta_actual))
        u1 = u1 / tilt_factor

        torques = self.att_ctrl.compute(
            self.state[6:9],
            np.array([phi_des, theta_des, 0.0]),
            self.state[9:12]
        )

        cmd = ThrustAndTorque()
        cmd.collective_thrust = float(u1)
        cmd.torque.x = float(torques[0])
        cmd.torque.y = float(torques[1])
        cmd.torque.z = float(torques[2])
        self.cmd_pub.publish(cmd)

        # Print status every 0.5 s.
        if now - self.last_debug_time >= 0.5:
            self.last_debug_time = now
            tracking_err = np.linalg.norm(self.state[0:3] - desired_pos)
            self.get_logger().info(
                f't={elapsed:.1f}s  '
                f'pos=({self.state[0]:.2f},{self.state[1]:.2f},{self.state[2]:.2f})  '
                f'des=({desired_pos[0]:.2f},{desired_pos[1]:.2f},{desired_pos[2]:.2f})  '
                f'err={tracking_err:.3f}m  '
                f'phi_cmd={np.degrees(phi_des):.1f} phi_act={np.degrees(phi_actual):.1f}  '
                f'tht_cmd={np.degrees(theta_des):.1f} tht_act={np.degrees(theta_actual):.1f}  '
                f'u1={u1:.3f}N'
            )

        # Publish pose and append to path for RViz.
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.state[0])
        pose.pose.position.y = float(self.state[1])
        pose.pose.position.z = float(self.state[2])
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
