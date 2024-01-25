import numpy as np
import time
import cutils

class Controller2D(object):
    def __init__(self):
        self.vars                   = cutils.CUtils()
        # self._lookahead_distance    = 2.0
        # self._current_x             = 0
        # self._current_y             = 0
        # self._current_yaw           = 0
        self._current_speed         = 0
        self._desired_speed         = 0
        # self._current_frame         = 0
        self._current_timestamp     = 0
        # self._start_control_loop    = False
        self._set_throttle          = 0
        # self._set_brake             = 0
        # self._set_steer             = 0
        # self._waypoints             = waypoints
        # self._conv_rad_to_steer     = 180.0 / 70.0 / np.pi
        self._pi                    = np.pi
        self._2pi                   = 2.0 * np.pi
        self.currenttime = self.current_time_in_nanoseconds()
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('error_previous', 0.0)
        self.vars.create_var('integral_error_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)

    def update_values(self, speed):
        self._current_speed     = speed
        self.currenttime = self.current_time_in_nanoseconds()

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle*100
        return self._set_throttle

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def current_time_in_nanoseconds(self):
        # Get current epoch time in seconds
        epoch_time_seconds = time.time()

        # Convert seconds to nanoseconds
        epoch_time_nanoseconds = int(epoch_time_seconds * 1e9)

        return epoch_time_nanoseconds


    def update_controls(self):
        v               = self._current_speed
        v_desired       = 10.0
        t               = self.currenttime
        throttle_output = 0
        
        kp = 1.65971
        ki = 0.00007
        kd = 1.710

        throttle_output = 0
        st = (t - self.vars.t_previous)/(1e9)
        print("current_time: ",t)
        print("previous: ",self.vars.t_previous)
        print("difference: ",st*100)
        print("velocity ",v)

        # error term
        e_v = v_desired - v

        # I
        inte_v = self.vars.integral_error_previous + e_v 

        # D
        derivate = (e_v - self.vars.error_previous)

        acc = kp * e_v + ki * inte_v + kd * derivate
        print("eeeeeeeeeee: ",acc)

        if acc > 0:
            throttle_output = (np.tanh(acc) + 1)/2
            # throttle_output = max(0.0, min(1.0, throttle_output))
            if throttle_output - self.vars.throttle_previous > 0.1:
                throttle_output = self.vars.throttle_previous + 0.1
        else:
            throttle_output = 0
        print("dddddd: ",throttle_output)


        
        ######################################################
        # SET CONTROLS OUTPUT
        ######################################################
        th = self.set_throttle(throttle_output)  # in percent (0 to 1)
        # self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
        # self.set_brake(brake_output)        # in percent (0 to 1)

        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.throttle_previous = throttle_output
        self.vars.t_previous = t
        self.vars.error_previous = e_v
        self.vars.integral_error_previous = inte_v

        return th