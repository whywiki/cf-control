# =============================================================================
# drone_sim.py
#
# Drone simulator that runs in Python. It models a small drone
# (Crazyflie) using the real equations of motion and flies it
# in a (well kind of :/) circle using the same PD controllers
# used on the real drone.
#
# The flow is:
#   1. Set up a drone with real properties (mass, inertia, etc.)
#   2. Each timestep: calculate where the drone should be on a circle
#   3. Run the position controller -> get thrust + desired tilt angles
#   4. Run the attitude controller -> get torques
#   5. Step the physics simulation forward in time
#   6. Publish the new position over ROS 2
# =============================================================================

import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


# =============================================================================
# DroneSim
#
# The physics engine. Stores the drone state and advances it forward in time
# using Newton's laws of motion.
#
# State vector has 12 elements:
#   [0:3]  - position in world frame (x, y, z) in meters
#   [3:6]  - velocity in world frame (vx, vy, vz) in m/s
#   [6:9]  - orientation angles (roll, pitch, yaw) in radians
#   [9:12] - angular velocity (p, q, r) in rad/s
# =============================================================================
class DroneSim:
    def __init__(self):
        self.mass = 0.025          # Total mass in kg
        self.g    = 9.81           # Gravity in m/s^2

        # Moments of inertia - how hard it is to spin the drone around each axis.
        # Values taken from the Crazyflie hardware datasheet.
        self.Ixx  = 16.571710e-6   # Resistance to rolling (left/right tilt)
        self.Iyy  = 16.655602e-6   # Resistance to pitching (forward/backward tilt)
        self.Izz  = 29.261652e-6   # Resistance to yawing (spinning on the spot)

        # Start on the ground, not moving.
        self.state = np.zeros(12)

    def step(self, T, tau_roll, tau_pitch, tau_yaw, dt):
        """
        Advance the drone physics by one small time step.

        Inputs:
            T         - total thrust force in Newtons (pointing up through the drone)
            tau_roll  - torque around the roll axis in N*m
            tau_pitch - torque around the pitch axis in N*m
            tau_yaw   - torque around the yaw axis in N*m
            dt        - timestep length in seconds

        The rotation matrix and angular momentum equations below are taken
        from drone dynamics literature - not derived here.
        """
        phi, theta, psi = self.state[6:9]   # Roll, pitch, yaw
        p, q, r = self.state[9:12]          # Angular velocity around each body axis

        # Rotation matrix: converts vectors from body frame to world frame.
        # Standard aerospace ZYX convention - taken from flight-dynamics textbooks.
        R = np.array([
            [np.cos(psi)*np.cos(theta),
             np.cos(psi)*np.sin(theta)*np.sin(phi) - np.sin(psi)*np.cos(phi),
             np.cos(psi)*np.sin(theta)*np.cos(phi) + np.sin(psi)*np.sin(phi)],
            [np.sin(psi)*np.cos(theta),
             np.sin(psi)*np.sin(theta)*np.sin(phi) + np.cos(psi)*np.cos(phi),
             np.sin(psi)*np.sin(theta)*np.cos(phi) - np.cos(psi)*np.sin(phi)],
            [-np.sin(theta), np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)]
        ])

        # Net linear acceleration: rotate thrust into world frame, subtract gravity.
        acc = (R @ np.array([0.0, 0.0, T]) + np.array([0.0, 0.0, -self.mass*self.g])) / self.mass

        # Angular acceleration via Euler's rotation equations.
        # The "omega x (I * omega)" term is the gyroscopic effect - taken from
        # classical mechanics, not derived here.
        I     = np.diag([self.Ixx, self.Iyy, self.Izz])
        omega = np.array([p, q, r])
        tau   = np.array([tau_roll, tau_pitch, tau_yaw])
        ang_acc = np.linalg.solve(I, tau - np.cross(omega, I @ omega))

        # Kinematic matrix W: converts body angular rates (p, q, r) into
        # Euler angle rates. Not the same thing when the drone is tilted.
        # Formula taken from flight-dynamics textbooks.
        W = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi),               -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta),  np.cos(phi)/np.cos(theta)],
        ])

        # Euler integration: new = old + rate * dt
        self.state[0:3]  += self.state[3:6] * dt
        self.state[3:6]  += acc * dt
        self.state[6:9]  += W @ omega * dt
        self.state[9:12] += ang_acc * dt


# =============================================================================
# AttitudeController
#
# Makes the drone rotate toward desired tilt angles.
# PD law: correct angle error, resist spinning too fast.
# =============================================================================
class AttitudeController:
    def __init__(self):
        self.kR = 0.005   # Angle correction strength
        self.kX = 0.002   # Damping

    def compute(self, current_angles, desired_angles, angular_velocity):
        return -self.kR * (current_angles - desired_angles) - self.kX * angular_velocity


# =============================================================================
# PositionController
#
# Computes thrust and desired tilt angles to reach a target position.
# Softer gains than the real drone - the sim drone is lighter and more agile.
# =============================================================================
class PositionController:
    def __init__(self):
        self.kx     = 0.5
        self.kx_dot = 1.0
        self.ky     = 0.5
        self.ky_dot = 1.0
        self.kz     = 1.0
        self.kz_dot = 2.0
        self.mass = 0.025
        self.g    = 9.81
        self.max_angle = np.radians(30)   # Never tilt more than 30 degrees

    def compute(self, state, desired_pos, desired_vel=None):
        x, y, z    = state[0:3]
        vx, vy, vz = state[3:6]
        dx, dy, dz = desired_pos

        if desired_vel is None:
            desired_vel = np.zeros(3)
        dvx, dvy, dvz = desired_vel

        # Thrust to reach target altitude.
        u1 = self.mass * (self.g
                          - self.kz     * (z  - dz)
                          - self.kz_dot * (vz - dvz))

        # Tilt angles from horizontal position errors.
        phi_des   = np.clip(+(1/self.g) * (self.ky * (y - dy) + self.ky_dot * (vy - dvy)),
                            -self.max_angle, self.max_angle)
        theta_des = np.clip(-(1/self.g) * (self.kx * (x - dx) + self.kx_dot * (vx - dvx)),
                            -self.max_angle, self.max_angle)

        return u1, phi_des, theta_des


# =============================================================================
# Main: run the simulation loop
# =============================================================================
if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('drone_sim')

    pose_pub = node.create_publisher(PoseStamped, '/pose', 10)
    path_pub = node.create_publisher(Path, '/path', 10)

    drone    = DroneSim()
    att_ctrl = AttitudeController()
    pos_ctrl = PositionController()

    r     = 0.5    # Circle radius in meters
    omega = 0.5    # Angular speed in rad/s

    # Start on the circle at angle 0, already at cruising altitude.
    drone.state[0] = r
    drone.state[2] = 1.0

    path_msg = Path()
    path_msg.header.frame_id = 'map'

    i = 0
    while rclpy.ok():
        t = i * 0.01   # Simulation time in seconds

        # Circle reference: x = r*cos(wt), y = r*sin(wt).
        # Velocity is the time-derivative of position.
        desired_pos = np.array([r*np.cos(omega*t), r*np.sin(omega*t), 1.0])
        desired_vel = np.array([-r*omega*np.sin(omega*t), r*omega*np.cos(omega*t), 0.0])

        u1, phi_des, theta_des = pos_ctrl.compute(drone.state, desired_pos, desired_vel)
        torques = att_ctrl.compute(
            drone.state[6:9],
            np.array([phi_des, theta_des, 0.0]),
            drone.state[9:12]
        )
        drone.step(u1, torques[0], torques[1], torques[2], 0.01)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = float(drone.state[0])
        pose.pose.position.y = float(drone.state[1])
        pose.pose.position.z = float(drone.state[2])
        pose.pose.orientation.w = 1.0

        pose_pub.publish(pose)
        path_msg.poses.append(pose)
        path_msg.header.stamp = pose.header.stamp
        path_pub.publish(path_msg)

        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.01)
        i += 1

    node.destroy_node()
    rclpy.shutdown()
