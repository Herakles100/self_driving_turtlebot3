#!/usr/bin/env python3
import time


class PID:
    def __init__(self, Kp, Ki, Kd):
        """
            Parameters
            -----------
            Kp: float
                The proportional gain.
            Ki: float
                The integral gain.
            Kd: float
                The derivative gain.
            origin_time: float
                The origin time.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.origin_time = time.time()

        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_time = self.origin_time
        self.previous_error = 0.0

    def update(self, error):
        """
            Update the PID loop with nonuniform time step size.

            Parameters
            -----------
            error: float
                Error since last call (state - target).
        """
        current_time = time.time()

        dt = current_time - self.previous_time

        if dt <= 0:
            return 0

        de = error - self.previous_error

        self.Cp = error
        self.Ci += error * dt
        self.Cd = de / dt

        self.previous_time = current_time
        self.previous_error = error

        output = self.Kp * self.Cp + self.Ki * self.Ci + self.Kd * self.Cd

        return -output
