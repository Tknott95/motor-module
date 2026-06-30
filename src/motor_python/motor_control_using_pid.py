"""AK60-6 PID controller using CubeMars Force Control Mode (MIT) only."""

import time

import numpy as np
from loguru import logger

from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN
from motor_python.definitions import LowPassFilterConfig, PIDConfig
from motor_python.pid_controller import PIDController


class PIDMotorController:
    """PID Controller methods specific for the motor.

    High-level wrapper around CubeMars motor using PID position control.
    Connects to the motor, checks communication, and provides methods to move to a target position using PID control.
    """

    def __init__(self, motor: CubeMarsAK606v3CAN):
        self.motor = motor

        filter_config = LowPassFilterConfig()

        self.pid = PIDController(
            pid_config=PIDConfig(
                proportional_gain=2.0,
                integral_gain=0.0,
                derivative_gain=0.3,
            ),
            filter_config=filter_config,
        )

    def move_to(
        self,
        target_degrees: float,
        duration: float = 2.0,
        rate_hz: int = 10,
        kp: float = 0.0,
        kd: float = 0.0,
    ):
        """Move motor to a target position using PID control.

        :param target_degrees: target angle in degrees
        :param velocity_rad_s: desired velocity in radians per second
        :param duration: how long to run control loop
        :param rate_hz: control frequency
        """
        logger.info(
            f"Moving to {target_degrees} degrees for {duration} seconds at {rate_hz} Hz"
        )
        dt = 1.0 / rate_hz
        start_time = time.time()

        target_rad = np.radians(target_degrees)

        while time.time() - start_time < duration:
            current_position = self.motor.get_position()
            if current_position is None:
                continue

            current_position_rad = np.radians(current_position)

            error = target_rad - current_position_rad
            if abs(error) < 0.01:  # radians
                logger.info(f"Target reached within tolerance: {current_position} deg")
                break

            now = time.monotonic()

            # --- PID compute (output in rad/s) ---
            velocity_cmd = self.pid.compute_output(
                timestamp=now,
                motor_reference=target_rad,
                motor_position=current_position_rad,
            )

            MAX_VELOCITY_RAD_S = PIDConfig.output_limits
            if MAX_VELOCITY_RAD_S is not None:
                velocity_cmd = np.clip(
                    velocity_cmd, MAX_VELOCITY_RAD_S[0], MAX_VELOCITY_RAD_S[1]
                )

            self.motor.set_mit_mode(
                pos_rad=target_rad,
                vel_rad_s=velocity_cmd,
                kp=kp,
                kd=kd,
            )

            velocity_cmd_deg_s = np.degrees(velocity_cmd)

            logger.info(
                f"Current = {current_position:.2f} deg | Target = {target_degrees:.2f} deg | "
                f"Error = {np.degrees(error):.2f} deg | PID vel = {velocity_cmd_deg_s:.2f} deg/s"
            )

            time.sleep(dt)

    def hold(self, target_degrees: float, kp: float = 20.0, kd: float = 1.0):
        """Hold motor at target position indefinitely."""
        while True:
            self.motor.set_mit_mode(
                pos_rad=np.radians(target_degrees),
                vel_rad_s=0.0,
                kp=kp,
                kd=kd,
            )
            time.sleep(0.01)

    def stop(self):
        """Stop the motor."""
        self.motor.stop()
