#!/usr/bin/env python3
"""
Converts /cmd_vel_smoothed (Twist) to PACMod2 steering, accel, and brake commands.
Applies the bicycle model internally (Twist -> road wheel angle) then runs the
PACMod2 speed PID and steering rate limiter.
Uses /pacmod/vehicle_speed_rpt for speed feedback.
Mirrors the logic in pacmod_twist_bridge_node.cpp.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pacmod2_msgs.msg import SystemCmdFloat, SystemCmdInt, PositionWithSpeed, VehicleSpeedRpt, GlobalRpt


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PID:
    def __init__(self, kp, ki, kd, i_min=-1.0, i_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_min, self.i_max = i_min, i_max
        self.integral = 0.0
        self.prev_err = None

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def step(self, err, dt):
        if dt <= 0.0:
            return 0.0
        self.integral = _clamp(self.integral + err * dt, self.i_min, self.i_max)
        derr = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * derr


class CmdVelToPacmod2(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_pacmod2')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed_rpt_topic', '/pacmod/vehicle_speed_rpt')

        # Vehicle geometry (bicycle model)
        self.declare_parameter('wheelbase_m', 3.4)
        self.declare_parameter('min_speed_for_steer_mps', 0.2)  # avoid w/v blow-up near zero

        # Steering geometry
        self.declare_parameter('steer_ratio', 16.0)
        self.declare_parameter('max_steer_wheel_rad', 10.9)
        self.declare_parameter('max_steer_rate_rad_s', 3.29)   # PACMod hardware limit

        # Accel PID
        self.declare_parameter('kp_accel', 0.40)
        self.declare_parameter('ki_accel', 0.03)
        self.declare_parameter('kd_accel', 0.00)
        self.declare_parameter('accel_i_min', 0.0)
        self.declare_parameter('accel_i_max', 0.30)

        # Brake PID
        self.declare_parameter('kp_brake', 4.0) #2.0
        self.declare_parameter('ki_brake', 0.40) #0.25
        self.declare_parameter('kd_brake', 0.00)
        self.declare_parameter('brake_i_min', 0.0)
        self.declare_parameter('brake_i_max', 0.50)

        # Command limits
        self.declare_parameter('accel_min', 0.22)
        self.declare_parameter('accel_max', 0.65)
        self.declare_parameter('brake_min', 0.30) # 0.20
        self.declare_parameter('brake_max', 0.75) # 0.75

        # Slew limits (command units per second)
        self.declare_parameter('accel_slew_per_s', 0.35)
        self.declare_parameter('brake_slew_per_s', 0.60)

        # Stiction compensation
        self.declare_parameter('stiction_speed_thresh_mps', 0.5)
        self.declare_parameter('accel_stiction_boost', 0.12)

        # Stop behaviour
        self.declare_parameter('stop_speed_thresh_mps', 0.10) # 0.05
        self.declare_parameter('stop_brake_hold', 0.75) #0.75

        # Safety
        self.declare_parameter('cmd_timeout_s', 0.25)

        # Shift
        self.declare_parameter('send_shift', True)
        self.declare_parameter('shift_gear', 3)

        # State
        self._v_meas = 0.0
        self._v_meas_valid = False
        self._v_cmd = 0.0
        self._steer_angle_cmd = 0.0       # road wheel angle (rad)
        self._steer_angle_vel_cmd = 0.0   # road wheel rate (rad/s), 0 = use param
        self._last_steer_cmd = 0.0
        self._last_accel_cmd = 0.0
        self._last_brake_cmd = 0.0
        self._pacmod_enabled = False
        self._already_sent_shift = False
        self._last_time = self.get_clock().now()
        self._last_cmd_time = self.get_clock().now()

        self._accel_pid = PID(
            self.get_parameter('kp_accel').value,
            self.get_parameter('ki_accel').value,
            self.get_parameter('kd_accel').value,
            i_min=self.get_parameter('accel_i_min').value,
            i_max=self.get_parameter('accel_i_max').value,
        )
        self._brake_pid = PID(
            self.get_parameter('kp_brake').value,
            self.get_parameter('ki_brake').value,
            self.get_parameter('kd_brake').value,
            i_min=self.get_parameter('brake_i_min').value,
            i_max=self.get_parameter('brake_i_max').value,
        )

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        speed_topic = self.get_parameter('speed_rpt_topic').value

        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
        self.create_subscription(VehicleSpeedRpt, speed_topic, self._speed_cb, 10)
        self.create_subscription(GlobalRpt, '/pacmod/global_rpt', self._enable_cb, 10)

        self._steer_pub = self.create_publisher(PositionWithSpeed, '/pacmod/steering_cmd', 10)
        self._accel_pub = self.create_publisher(SystemCmdFloat, '/pacmod/accel_cmd', 10)
        self._brake_pub = self.create_publisher(SystemCmdFloat, '/pacmod/brake_cmd', 10)
        self._shift_pub = self.create_publisher(SystemCmdInt, '/pacmod/shift_cmd', 10)

        self.create_timer(0.02, self._loop)  # 50 Hz
        self.get_logger().info(f'cmd_vel_to_pacmod2: {cmd_topic} -> /pacmod/{{steering,accel,brake}}_cmd')

    def _cmd_cb(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        L = self.get_parameter('wheelbase_m').value
        steer_ratio = self.get_parameter('steer_ratio').value
        max_road_wheel = self.get_parameter('max_steer_wheel_rad').value / steer_ratio

        self._v_cmd = v
        self._last_cmd_time = self.get_clock().now()

        # Bicycle model: road_wheel = atan(L * w / v)
        # Zero steering when near-stopped to avoid w/v singularity
        if abs(v) < self.get_parameter('min_speed_for_steer_mps').value:
            self._steer_angle_cmd = 0.0
        else:
            self._steer_angle_cmd = _clamp(math.atan(L * w / v), -max_road_wheel, max_road_wheel)

        self._steer_angle_vel_cmd = 0.0  # no per-message rate from Twist; use param

    def _speed_cb(self, msg: VehicleSpeedRpt):
        self._v_meas = float(msg.vehicle_speed)
        self._v_meas_valid = bool(msg.vehicle_speed_valid)

    def _enable_cb(self, msg: GlobalRpt):
        self._pacmod_enabled = bool(msg.enabled)
        if not self._pacmod_enabled:
            self._already_sent_shift = False

    def _loop(self):
        if not self._pacmod_enabled:
            return

        if self.get_parameter('send_shift').value and not self._already_sent_shift:
            m = SystemCmdInt()
            m.command = int(self.get_parameter('shift_gear').value)
            self._shift_pub.publish(m)
            self._already_sent_shift = True

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        self._last_time = now

        if not self._v_meas_valid:
            self._pub_accel(0.0)
            self._pub_brake(_clamp(self.get_parameter('stop_brake_hold').value, 0.0,
                                   self.get_parameter('brake_max').value))
            self._pub_steer(0.0, self.get_parameter('max_steer_rate_rad_s').value)
            return

        # Command timeout -> stop
        cmd_age = (now - self._last_cmd_time).nanoseconds * 1e-9
        v_cmd = self._v_cmd if cmd_age <= self.get_parameter('cmd_timeout_s').value else 0.0

        # --- Steering ---
        steer_ratio = self.get_parameter('steer_ratio').value
        max_steer = self.get_parameter('max_steer_wheel_rad').value
        max_rate = self.get_parameter('max_steer_rate_rad_s').value

        target_steer = _clamp(steer_ratio * self._steer_angle_cmd, -max_steer, max_steer)

        # Per-message rate if provided, else param (convert road wheel rate to steering wheel rate)
        steer_rate = (self._steer_angle_vel_cmd * steer_ratio
                      if self._steer_angle_vel_cmd > 0.0
                      else max_rate)

        max_delta = steer_rate * max(dt, 1e-3)
        steer_cmd = _clamp(target_steer,
                           self._last_steer_cmd - max_delta,
                           self._last_steer_cmd + max_delta)
        self._last_steer_cmd = steer_cmd
        self._pub_steer(steer_cmd, steer_rate)

        # --- Speed control ---
        stop_thresh = self.get_parameter('stop_speed_thresh_mps').value
        brake_hold = self.get_parameter('stop_brake_hold').value
        accel_min = self.get_parameter('accel_min').value
        accel_max = self.get_parameter('accel_max').value
        brake_min = self.get_parameter('brake_min').value
        brake_max = self.get_parameter('brake_max').value

        if abs(v_cmd) < 1e-3 and abs(self._v_meas) < stop_thresh:
            self._accel_pid.reset()
            self._pub_accel(0.0)
            self._pub_brake(_clamp(brake_hold, brake_min, brake_max))
            self._last_accel_cmd = 0.0
            self._last_brake_cmd = _clamp(brake_hold, brake_min, brake_max)
            return

        e = v_cmd - self._v_meas

        if e > 0.0:
            self._brake_pid.reset()
            u = self._accel_pid.step(e, dt)
            bias = accel_min
            if abs(self._v_meas) < self.get_parameter('stiction_speed_thresh_mps').value:
                bias += self.get_parameter('accel_stiction_boost').value
            accel_cmd = _clamp(bias + u, accel_min, accel_max)
            brake_cmd = 0.0
        else:
            self._accel_pid.reset()
            eb = -e
            u = self._brake_pid.step(eb, dt)
            accel_cmd = 0.0
            brake_cmd = _clamp(u, brake_min, brake_max)

        # Slew limits
        accel_slew = self.get_parameter('accel_slew_per_s').value * dt
        brake_slew = self.get_parameter('brake_slew_per_s').value * dt
        accel_cmd = _clamp(accel_cmd, self._last_accel_cmd - accel_slew, self._last_accel_cmd + accel_slew)
        brake_cmd = _clamp(brake_cmd, self._last_brake_cmd - brake_slew, self._last_brake_cmd + brake_slew)

        if accel_cmd > 0.30:
            brake_cmd = 0.0
        if brake_cmd > 0.0:
            accel_cmd = 0.0

        self._last_accel_cmd = accel_cmd
        self._last_brake_cmd = brake_cmd
        self._pub_accel(accel_cmd)
        self._pub_brake(brake_cmd)

    def _pub_steer(self, angle: float, rate: float):
        m = PositionWithSpeed()
        m.angular_position = float(angle)
        m.angular_velocity_limit = float(abs(rate))
        self._steer_pub.publish(m)

    def _pub_accel(self, v: float):
        m = SystemCmdFloat()
        m.command = float(v)
        self._accel_pub.publish(m)

    def _pub_brake(self, v: float):
        m = SystemCmdFloat()
        m.command = float(v)
        self._brake_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(CmdVelToPacmod2())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
