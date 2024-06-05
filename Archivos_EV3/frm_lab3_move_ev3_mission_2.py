#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, LargeMotor
import math
import time
import _thread
from logging import getLogger

log = getLogger(__name__)

class DeviceNotFound(Exception):
    pass
class DeviceNotDefined(Exception):
    pass
class ThreadNotRunning(Exception):
    pass


class MoveTankMission2(MoveTank):

    def __init__(self,
                 left_motor_port,
                 right_motor_port,
                 wheel_class,
                 wheel_distance_mm,
                 desc=None,
                 motor_class=LargeMotor):

        MoveTank.__init__(self, left_motor_port, right_motor_port, desc, motor_class)
        self.wheel = wheel_class()
        self.wheel_distance_mm = wheel_distance_mm


        # The circumference of the circle made if this robot were to rotate in place
        self.circumference_mm = self.wheel_distance_mm * math.pi

        self.min_circle_radius_mm = self.wheel_distance_mm / 2

        # odometry variables
        self.x_pos_mm = 0.0  # robot X position in mm
        self.y_pos_mm = 0.0  # robot Y position in mm
        self.odometry_thread_run = False
        self.theta = 0.0
        
        # Detection variables
        self._ultrasonic_s = None
        self._uts_motor = None
        self._touch_sensor = None
        self.front_safe_distance = 20.0 #cm

    
    # Ultrasonic sensor
    @property
    def ultrasonic_s(self):
        return self._ultrasonic_s

    @ultrasonic_s.setter
    def ultrasonic_s(self, ultrasonic_s):
        self._ultrasonic_s = ultrasonic_s

    # Motor used to rotate ultrasonic sensor
    @property
    def uts_motor(self):
        return self._uts_motor

    @uts_motor.setter
    def uts_motor(self, uts_motor):
        self._uts_motor = uts_motor

    # Touch sensor used to detect goal
    @property
    def touch_sensor(self):
        return self._touch_sensor

    @touch_sensor.setter
    def touch_sensor(self, touch_sensor):
        self._touch_sensor = touch_sensor

    def on_for_distance(self, speed, distance_mm, brake=True, block=True):
        """
        Drive in a straight line for ``distance_mm``
        """
        rotations = distance_mm / self.wheel.circumference_mm
        log.debug("%s: on_for_rotations distance_mm %s, rotations %s, speed %s" % (self, distance_mm, rotations, speed))

        MoveTank.on_for_rotations(self, speed, speed, rotations, brake, block)

    def _on_arc(self, speed, radius_mm, distance_mm, brake, block, arc_right):
        """
        Drive in a circle with 'radius' for 'distance'
        """

        if radius_mm < self.min_circle_radius_mm:
            raise ValueError("{}: radius_mm {} is less than min_circle_radius_mm {}".format(
                self, radius_mm, self.min_circle_radius_mm))

        # The circle formed at the halfway point between the two wheels is the
        # circle that must have a radius of radius_mm
        circle_outer_mm = 2 * math.pi * (radius_mm + (self.wheel_distance_mm / 2))
        circle_middle_mm = 2 * math.pi * radius_mm
        circle_inner_mm = 2 * math.pi * (radius_mm - (self.wheel_distance_mm / 2))

        if arc_right:
            # The left wheel is making the larger circle and will move at 'speed'
            # The right wheel is making a smaller circle so its speed will be a fraction of the left motor's speed
            left_speed = speed
            right_speed = float(circle_inner_mm / circle_outer_mm) * left_speed

        else:
            # The right wheel is making the larger circle and will move at 'speed'
            # The left wheel is making a smaller circle so its speed will be a fraction of the right motor's speed
            right_speed = speed
            left_speed = float(circle_inner_mm / circle_outer_mm) * right_speed

        log.debug("%s: arc %s, radius %s, distance %s, left-speed %s, right-speed %s" %
                  (self, "right" if arc_right else "left", radius_mm, distance_mm, left_speed, right_speed))
        log.debug("%s: circle_outer_mm %s, circle_middle_mm %s, circle_inner_mm %s" %
                  (self, circle_outer_mm, circle_middle_mm, circle_inner_mm))

        # We know we want the middle circle to be of length distance_mm so
        # calculate the percentage of circle_middle_mm we must travel for the
        # middle of the robot to travel distance_mm.
        circle_middle_percentage = float(distance_mm / circle_middle_mm)

        # Now multiple that percentage by circle_outer_mm to calculate how
        # many mm the outer wheel should travel.
        circle_outer_final_mm = circle_middle_percentage * circle_outer_mm

        outer_wheel_rotations = float(circle_outer_final_mm / self.wheel.circumference_mm)
        outer_wheel_degrees = outer_wheel_rotations * 360

        log.debug("%s: arc %s, circle_middle_percentage %s, circle_outer_final_mm %s, " %
                  (self, "right" if arc_right else "left", circle_middle_percentage, circle_outer_final_mm))
        log.debug("%s: outer_wheel_rotations %s, outer_wheel_degrees %s" %
                  (self, outer_wheel_rotations, outer_wheel_degrees))

        MoveTank.on_for_degrees(self, left_speed, right_speed, outer_wheel_degrees, brake, block)

    def on_arc_right(self, speed, radius_mm, distance_mm, brake=True, block=True):
        """
        Drive clockwise in a circle with 'radius_mm' for 'distance_mm'
        """
        self._on_arc(speed, radius_mm, distance_mm, brake, block, True)

    def on_arc_left(self, speed, radius_mm, distance_mm, brake=True, block=True):
        """
        Drive counter-clockwise in a circle with 'radius_mm' for 'distance_mm'
        """
        self._on_arc(speed, radius_mm, distance_mm, brake, block, False)

    def turn_degrees(self, speed, degrees, brake=True, block=True, error_margin=2, use_gyro=False):
        """
        Rotate in place ``degrees``. Both wheels must turn at the same speed for us
        to rotate in place.  If the following conditions are met the GryoSensor will
        be used to improve the accuracy of our turn:
        - ``use_gyro``, ``brake`` and ``block`` are all True
        - A GyroSensor has been defined via ``self.gyro = GyroSensor()``
        """
        def final_angle(init_angle, degrees):
            result = init_angle - degrees

            while result <= -360:
                result += 360

            while result >= 360:
                result -= 360

            if result < 0:
                result += 360

            return result

        # use the gyro to check that we turned the correct amount?
        use_gyro = bool(use_gyro and block and brake)
        if use_gyro and not self._gyro:
            raise DeviceNotDefined(
                "The 'gyro' variable must be defined with a GyroSensor. Example: tank.gyro = GyroSensor()")

        if use_gyro:
            angle_init_degrees = self._gyro.circle_angle()
        else:
            angle_init_degrees = math.degrees(self.theta)

        angle_target_degrees = final_angle(angle_init_degrees, degrees)

        log.info("%s: turn_degrees() %d degrees from %s to %s" %
                 (self, degrees, angle_init_degrees, angle_target_degrees))

        # The distance each wheel needs to travel
        distance_mm = (abs(degrees) / 360) * self.circumference_mm

        # The number of rotations to move distance_mm
        rotations = distance_mm / self.wheel.circumference_mm

        # If degrees is positive rotate clockwise
        if degrees > 0:
            MoveTank.on_for_rotations(self, speed, speed * -1, rotations, brake, block)

        # If degrees is negative rotate counter-clockwise
        else:
            MoveTank.on_for_rotations(self, speed * -1, speed, rotations, brake, block)

        if use_gyro:
            angle_current_degrees = self._gyro.circle_angle()

            # This can happen if we are aiming for 2 degrees and overrotate to 358 degrees
            # We need to rotate counter-clockwise
            if 90 >= angle_target_degrees >= 0 and 270 <= angle_current_degrees <= 360:
                degrees_error = (angle_target_degrees + (360 - angle_current_degrees)) * -1

            # This can happen if we are aiming for 358 degrees and overrotate to 2 degrees
            # We need to rotate clockwise
            elif 360 >= angle_target_degrees >= 270 and 0 <= angle_current_degrees <= 90:
                degrees_error = angle_current_degrees + (360 - angle_target_degrees)

            # We need to rotate clockwise
            elif angle_current_degrees > angle_target_degrees:
                degrees_error = angle_current_degrees - angle_target_degrees

            # We need to rotate counter-clockwise
            else:
                degrees_error = (angle_target_degrees - angle_current_degrees) * -1

            log.info("%s: turn_degrees() ended up at %s, error %s, error_margin %s" %
                     (self, angle_current_degrees, degrees_error, error_margin))

            if abs(degrees_error) > error_margin:
                self.turn_degrees(speed, degrees_error, brake, block, error_margin, use_gyro)

    def turn_right(self, speed, degrees, brake=True, block=True, error_margin=2, use_gyro=False):
        """
        Rotate clockwise ``degrees`` in place
        """
        self.turn_degrees(speed, abs(degrees), brake, block, error_margin, use_gyro)

    def turn_left(self, speed, degrees, brake=True, block=True, error_margin=2, use_gyro=False):
        """
        Rotate counter-clockwise ``degrees`` in place
        """
        self.turn_degrees(speed, abs(degrees) * -1, brake, block, error_margin, use_gyro)

    def turn_to_angle(self, speed, angle_target_degrees, brake=True, block=True, error_margin=2, use_gyro=False):
        """
        Rotate in place to ``angle_target_degrees`` at ``speed``
        """
        if not self.odometry_thread_run:
            raise ThreadNotRunning("odometry_start() must be called to track robot coordinates")

        # Make both target and current angles positive numbers between 0 and 360
        while angle_target_degrees < 0:
            angle_target_degrees += 360

        angle_current_degrees = math.degrees(self.theta)

        while angle_current_degrees < 0:
            angle_current_degrees += 360

        # Is it shorter to rotate to the right or left
        # to reach angle_target_degrees?
        if angle_current_degrees > angle_target_degrees:
            turn_right = True
            angle_delta = angle_current_degrees - angle_target_degrees
        else:
            turn_right = False
            angle_delta = angle_target_degrees - angle_current_degrees

        if angle_delta > 180:
            angle_delta = 360 - angle_delta
            turn_right = not turn_right

        log.debug("%s: turn_to_angle %s, current angle %s, delta %s, turn_right %s" %
                  (self, angle_target_degrees, angle_current_degrees, angle_delta, turn_right))
        self.odometry_coordinates_log()

        if turn_right:
            self.turn_degrees(speed, abs(angle_delta), brake, block, error_margin, use_gyro)
        else:
            self.turn_degrees(speed, abs(angle_delta) * -1, brake, block, error_margin, use_gyro)

        self.odometry_coordinates_log()

    def odometry_coordinates_log(self):
        log.debug("%s: odometry angle %s at (%d, %d)" % (self, math.degrees(self.theta), self.x_pos_mm, self.y_pos_mm))

    def odometry_start(self, theta_degrees_start=90.0, x_pos_start=0.0, y_pos_start=0.0, sleep_time=0.005):  # 5ms
        """
        Ported from:
        http://seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm

        A thread is started that will run until the user calls odometry_stop()
        which will set odometry_thread_run to False
        """
        def _odometry_monitor():
            left_previous = 0
            right_previous = 0
            self.theta = math.radians(theta_degrees_start)  # robot heading
            self.x_pos_mm = x_pos_start  # robot X position in mm
            self.y_pos_mm = y_pos_start  # robot Y position in mm
            TWO_PI = 2 * math.pi
            self.odometry_thread_run = True

            while self.odometry_thread_run:

                # sample the left and right encoder counts as close together
                # in time as possible
                left_current = self.left_motor.position
                right_current = self.right_motor.position

                # determine how many ticks since our last sampling
                left_ticks = left_current - left_previous
                right_ticks = right_current - right_previous

                # Have we moved?
                if not left_ticks and not right_ticks:
                    if sleep_time:
                        time.sleep(sleep_time)
                    continue

                # update _previous for next time
                left_previous = left_current
                right_previous = right_current

                # rotations = distance_mm/self.wheel.circumference_mm
                left_rotations = float(left_ticks / self.left_motor.count_per_rot)
                right_rotations = float(right_ticks / self.right_motor.count_per_rot)

                # convert longs to floats and ticks to mm
                left_mm = float(left_rotations * self.wheel.circumference_mm)
                right_mm = float(right_rotations * self.wheel.circumference_mm)

                # calculate distance we have traveled since last sampling
                mm = (left_mm + right_mm) / 2.0

                # accumulate total rotation around our center
                self.theta += (right_mm - left_mm) / self.wheel_distance_mm

                # and clip the rotation to plus or minus 360 degrees
                self.theta -= float(int(self.theta / TWO_PI) * TWO_PI)

                # now calculate and accumulate our position in mm
                self.x_pos_mm += mm * math.cos(self.theta)
                self.y_pos_mm += mm * math.sin(self.theta)

                if sleep_time:
                    time.sleep(sleep_time)

        _thread.start_new_thread(_odometry_monitor, ())

        # Block until the thread has started doing work
        while not self.odometry_thread_run:
            pass

    def odometry_stop(self):
        """
        Signal the odometry thread to exit
        """

        if self.odometry_thread_run:
            self.odometry_thread_run = False

    def on_to_coordinates(self, speed, x_target_mm, y_target_mm, brake=True, block=True, bug2=True):
        """
        Drive to (``x_target_mm``, ``y_target_mm``) coordinates at ``speed``
        """
        if not self.odometry_thread_run:
            raise ThreadNotRunning("odometry_start() must be called to track robot coordinates")

        # stop moving
        self.off(brake='hold')
        
        self._uts_motor.on_to_position(speed=speed,position=0)

        x_delta = x_target_mm - self.x_pos_mm
        y_delta = y_target_mm - self.y_pos_mm

        # rotate in place so we are pointed straight at our target
        angle_target_radians = math.atan2(y_delta, x_delta)
        angle_target_degrees = math.degrees(angle_target_radians)
        self.turn_to_angle(speed, angle_target_degrees, brake=True, block=True)

        # drive in a straight line to the target coordinates
        distance_mm = math.sqrt(pow(self.x_pos_mm - x_target_mm, 2) + pow(self.y_pos_mm - y_target_mm, 2))
        self.on_for_distance(speed, distance_mm, brake, block)


# READ SENSORS
# self._touch_sensor.is_pressed
# self._uts_motor.on_to_position(speed=20,position=-90)
# self._ultrasonic_s.distance_centimeters
# self._gyro_sensor.angle
# self.theta