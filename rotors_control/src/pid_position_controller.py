#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# commit try 1
import rospy
import os
import numpy as np
from numpy.core.numeric import NaN
from mav_msgs.msg import Actuators

from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from rotors_control.cfg import pid_tuningsConfig as ConfigType
from tf.transformations import euler_from_quaternion

class RunningAverageFilter():
    """Simple Runnning Average Convolution Filter"""
    def __init__(self,magnitude):
        self.current=[]
        self.magnitude=magnitude
        self.data=[]
    def __call__(self, current):

        averaged_list= [ ]
        self.data.append(current)
        try:
            self.data.pop(1-self.magnitude)
        except IndexError:
            pass

        for i in range(len(current)-1):
            sum=0
            for j in range(len(self.data)-1):
                sum+=self.data[i][j]
            
            averaged_list.append( sum/self.magnitude)

        return averaged_list



class PID():
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=0.01,
        output_limits=(None, None),
        auto_mode=True,
        proportional_on_measurement=False,
        error_map=None,
        integral_windup_limit=None
    ):
        """
        Initialize a new PID controller.
        :param Kp: The value for the proportional gain Kp
        :param Ki: The value for the integral gain Ki
        :param Kd: The value for the derivative gain Kd
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time: The time in seconds which the controller should wait before generating
            a new output value. The PID works best when it is constantly called (eg. during a
            loop), but with a sample time set so that the time difference between each update is
            (close to) constant. If set to None, the PID will compute a new output value every time
            it is called.
        :param output_limits: The initial output limits to use, given as an iterable with 2
            elements, for example: (lower, upper). The output will never go below the lower limit
            or above the upper limit. Either of the limits can also be set to None to have no limit
            in that direction. Setting output limits also avoids integral windup, since the
            integral term will never be allowed to grow outside of the limits.
        :param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on
            the input directly rather than on the error (which is the traditional way). Using
            proportional-on-measurement avoids overshoot for some types of systems.
        :param error_map: Function to transform the error value in another constrained value.
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_input = None
        self._last_integralreset=0
        self.output_limits = output_limits


        self.integral_windup_limit=integral_windup_limit

        self.reset()

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.
        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).
        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = rospy.get_time()
        if dt is None:
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        error = self.setpoint - input_
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)

        # Check if must map the error
        if self.error_map is not None:

            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        #if self.integral_windup_limit is not None:
        #    self._integral = self.__clamp__(self._integral, (-1*self.integral_windup_limit,self.integral_windup_limit))  # Avoid integral windup
  
        
        # Alternative Integral Windup Avoidance
        if  self.integral_windup_limit is not None and abs(self._integral) > self.integral_windup_limit:
            sign=(self._integral>0)-(self._integral<0)     
            #self._integral= min(abs(self._integral)/(self.Ki*1.1), self.integral_windup_limit) * sign
            self._integral= self.integral_windup_limit * sign


        self._derivative = - self.Kd * d_input / dt
        self.time_data=[dt, bool(now - self._last_time), now, self._last_time]

        # Compute final output
        output = self._proportional + self._integral + self._derivative
        output = self.__clamp__(output, self.output_limits)
        self._integral=self._integral*0.8
        
        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now




        return output

    def __repr__(self):
        return (
            '{self.__class__.__name__}('
            'Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, '
            'setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, '
            'output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, '
            'proportional_on_measurement={self.proportional_on_measurement!r},'
            'error_map={self.error_map!r}'
            ')'
        ).format(self=self)

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def integral_term(self):
        """Returns current integral term."""
        return self._integral

    @integral_term.setter
    def integral_term(self, integral):
        """ Sets current integral value to the input value."""
        self._integral=integral

    def get_time_data(self):
        """ Sets current integral value to the input value."""
        return self.time_data

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not."""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller."""
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """
        Enable or disable the PID controller, optionally setting the last output value.
        This is useful if some system has been manually controlled and if the PID should take over.
        In that case, disable the PID by setting auto mode to False and later when the PID should
        be turned back on, pass the last output variable (the control variable) and it will be set
        as the starting I-term when the PID is set to auto mode.
        :param enabled: Whether auto mode should be enabled, True or False
        :param last_output: The last output, or the control variable, that the PID should start
            from when going from manual mode to auto mode. Has no effect if the PID is already in
            auto mode.
        """
        if enabled and not self._auto_mode:
            # Switching from manual mode to auto, reset
            self.reset()

            self._integral = last_output if (last_output is not None) else 0
            self._integral = self.__clamp__(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).
        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if (None not in limits) and (max_output < min_output):
            raise ValueError('lower limit must be less than upper limit')

        self._min_output = min_output
        self._max_output = max_output

        self._integral = self.__clamp__(self._integral, self.output_limits)
        self._last_output = self.__clamp__(self._last_output, self.output_limits)

    def reset(self):
        """
        Reset the PID controller internals.
        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._integral = self.__clamp__(self._integral, self.output_limits)

        self._last_time = rospy.get_time()
        self._last_output = None
        self._last_input = None

    def __clamp__(self, value, limits):

        lower, upper = limits

        if value is None:
            return None
        elif (upper is not None) and (value > upper):
            return upper
        elif (lower is not None) and (value < lower):
            return lower
        return value
   


class SubTrajectory():
    def __init__(self):
        

    
        self.Sub_to_Topic="command/trajectory"
        """Read in parameters."""
        self.enable = True
        if self.enable:
            self.start()
        else:
            self.stop()
        self.translation=Vector3()
        #initiate elements
        self.rotation=Vector3()
        #self.data=MultiDOFJointTrajectoryPoint()
        

    def callback(self, msg):
        """Handle subscriber data."""
        #print(msg)
        self.data=msg.points[0]
        
        self.translation=self.data.transforms[0].translation
        self.rotation=self.data.transforms[0].rotation
        
    def start(self):
        """Configure subscriber."""

        self.sub=rospy.Subscriber(self.Sub_to_Topic, MultiDOFJointTrajectory, self.callback)
        
    def stop(self):
        self.sub.unregister()



class SubIMU():
    def __init__(self):
        
        """Read in parameters."""
     
        self.Sub_to_Topic="ground_truth/imu"
        self.enable = True
        if self.enable:
            self.start()
        else:
            self.stop()
        self.orientation=Quaternion()
        self.angular_velocity=Vector3()
        self.linear_acceleration=Vector3()
   
    

    def callback(self, data):
        """Handle subscriber data."""
        
        self.orientation=data.orientation
    
        self.angular_velocity=data.angular_velocity
      
        self.linear_acceleration=data.linear_acceleration
        
    def start(self):
        """Configure subscriber."""

        self.sub=rospy.Subscriber(self.Sub_to_Topic, Imu, self.callback)
        
    def stop(self):

        self.sub.unregister()



class SubOdometry():
    def __init__(self):
        
        """Read in parameters."""

        self.Sub_to_Topic="odometry"

        self.enable = True

        if self.enable:
            self.start()
        else:
            self.stop()

        self.position=Point() 
        self.orientation=Quaternion()
         
        self.velocity=Vector3() 
        self.angular_velocity=Vector3() 


    def callback(self, data):
        """Handle subscriber data."""
        
       
        self.position=data.pose.pose.position #point
        self.orientation=data.pose.pose.orientation #point
         
        self.velocity=data.twist.twist.linear #vector3
        self.angular_velocity=data.twist.twist.angular #vector3
        

       
    def start(self):
        """Configure subscriber."""

        self.sub=rospy.Subscriber(self.Sub_to_Topic, Odometry, self.callback)
        
        
    def stop(self):
        self.sub.unregister()

    def __call__(self):

        
        self.start()

        return self.position, self.orientation, self.velocity, self.angular_velocity



class Controller():
    def __init__(self,Trajectory, IMU, ODOM ):
    
        self.enable=True
        self.angularvelocity=Vector3()
        self.orientation=Vector3()
        self.filter=RunningAverageFilter(3)


        self.update(Trajectory,IMU, ODOM)

        self.log=[]

        self.override_default_pid_vals =False
        self.rate_controller_tuning_mode =False
        self.manual_inputs=[0,0,0,0]

        self.calculate_hovering_throttle()

        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)
        
        self.initialize_pid()


    def initialize_pid(self):
        
        altitude_gain=rospy.search_param("altitude_gain")
        position_x_gain=rospy.search_param("position_x_gain")
        position_y_gain=rospy.search_param("position_y_gain")
        
        attitude_roll_gain=rospy.search_param("attitude_roll_gain")
        attitude_pitch_gain=rospy.search_param("attitude_pitch_gain")
        attitude_yaw_gain=rospy.search_param("attitude_yaw_gain")

        angular_rate_roll_gain=rospy.search_param("angular_rate_roll_gain")
        angular_rate_pitch_gain=rospy.search_param("angular_rate_pitch_gain")
        angular_rate_yaw_gain=rospy.search_param("angular_rate_yaw_gain")

        altitude_gain=rospy.get_param(altitude_gain)
        position_x_gain=rospy.get_param(position_x_gain)
        position_y_gain=rospy.get_param(position_y_gain)
        
        attitude_roll_gain=rospy.get_param(attitude_roll_gain)
        attitude_pitch_gain=rospy.get_param(attitude_pitch_gain)
        attitude_yaw_gain=rospy.get_param(attitude_yaw_gain)

        angular_rate_roll_gain=rospy.get_param(angular_rate_roll_gain)
        angular_rate_pitch_gain=rospy.get_param(angular_rate_pitch_gain)
        angular_rate_yaw_gain=rospy.get_param(angular_rate_yaw_gain)


        self.throttle_controller=PID(altitude_gain['P'], altitude_gain['I'], altitude_gain['D'], setpoint=0 , output_limits=altitude_gain["Output Limits"], integral_windup_limit=altitude_gain["Integral Term Limit"] )
        
        self.position_controller_x=PID(position_x_gain['P'], position_x_gain['I'], position_x_gain['D'], setpoint=0 , output_limits=position_x_gain["Output Limits"], integral_windup_limit=position_x_gain["Integral Term Limit"] )
        self.position_controller_y=PID(position_y_gain['P'], position_y_gain['I'], position_y_gain['D'], setpoint=0 , output_limits=position_y_gain["Output Limits"], integral_windup_limit=position_y_gain["Integral Term Limit"] )


        self.attitute_controller_roll=PID(attitude_roll_gain['P'], attitude_roll_gain['I'], attitude_roll_gain['D'],  setpoint=0 ,output_limits=attitude_roll_gain["Output Limits"], integral_windup_limit=attitude_roll_gain["Integral Term Limit"] )
        self.attitute_controller_pitch=PID(attitude_pitch_gain['P'], attitude_pitch_gain['I'], attitude_pitch_gain['D'],  setpoint=0  , output_limits=attitude_pitch_gain["Output Limits"], integral_windup_limit=attitude_pitch_gain["Integral Term Limit"])
        self.attitute_controller_yaw=PID(attitude_yaw_gain['P'], attitude_yaw_gain['I'], attitude_yaw_gain['D'], setpoint=0  , output_limits=attitude_yaw_gain["Output Limits"], integral_windup_limit=attitude_yaw_gain["Integral Term Limit"])

        self.rate_controller_roll=PID(angular_rate_roll_gain['P'] ,angular_rate_roll_gain['I'] , angular_rate_roll_gain['D'],  setpoint=0 , output_limits=angular_rate_roll_gain["Output Limits"], integral_windup_limit=angular_rate_roll_gain["Integral Term Limit"])
        self.rate_controller_pitch=PID(angular_rate_pitch_gain['P'],angular_rate_pitch_gain['I'] , angular_rate_pitch_gain['D'],  setpoint=0 , output_limits=angular_rate_pitch_gain["Output Limits"], integral_windup_limit=angular_rate_pitch_gain["Integral Term Limit"])
        self.rate_controller_yaw=PID(angular_rate_yaw_gain['P'], angular_rate_yaw_gain['I'], angular_rate_yaw_gain['D'],  setpoint=0    ,  output_limits=angular_rate_yaw_gain["Output Limits"], integral_windup_limit=angular_rate_yaw_gain["Integral Term Limit"])


    def update(self, Trajectory, IMU, ODOM ):
        #Get position vals


        self.desired_position=Trajectory.translation
    
        
        self.ThrottleDes=self.desired_position  # Vector3() object.
        self.YawDes=Trajectory.rotation.z

        (self.orientation.x, self.orientation.y, self.orientation.z) = euler_from_quaternion([IMU.orientation.x, IMU.orientation.y, IMU.orientation.z, IMU.orientation.w])
        self.angularvelocity=IMU.angular_velocity

        self.position=ODOM.position
        
        self.orientation.x, self.orientation.y, self.orientation.z, self.angularvelocity.x, self.angularvelocity.y, self.angularvelocity.z ==self.filter([self.orientation.x, self.orientation.y, self.orientation.z, self.angularvelocity.x, self.angularvelocity.y, self.angularvelocity.z])   


    def calculate_hovering_throttle(self):


        mass= rospy.search_param("mass")
        mass=rospy.get_param(mass)
        motor_constant=rospy.search_param("motor_constant")
        motor_constant=rospy.get_param(motor_constant)
        self.hovering_throttle=np.sqrt( (mass*9.81/4) / motor_constant) 
        
        self.hovering_throttle=865


    def altitude_controller(self):
        self.throttle_controller.setpoint= self.ThrottleDes.z
        ctrl_term=self.throttle_controller(self.position.z)
        throttle =  ctrl_term + self.hovering_throttle
        state= [self.ThrottleDes.z, self.position.z, throttle ]
        #print("altitude_desired: %f, position_z: %f, throttle: %f".format(self.ThrottleDes.z, self.position.z, throttle))

        dt, bool_, now, last_time= self.throttle_controller.get_time_data()
       # print("dt, bool_, now, last_time",d
       # t, bool_, now, last_time  )
        print(ctrl_term,self.ThrottleDes.z, self.position.z, throttle)
        return throttle, state

    def position_controller(self):


        self.position_controller_x.setpoint=self.desired_position.x
        V_X_Des   = self.position_controller_x(self.position.x)

        
        self.position_controller_y.setpoint=self.desired_position.y 
        V_Y_Des  = self.position_controller_y(self.position.y)


        # Apply a rotation matrix to desired positions. (Desired positions are w.r.t. global frame)
        theta= - self.orientation.z # Multiply by (-) due to difference between angle conventions. 

        PitchDes = np.cos(theta)*V_X_Des + np.sin(theta)*V_Y_Des
        RollDes= - np.sin(theta)*V_X_Des + np.cos(theta)*V_Y_Des

        state=[self.desired_position.x, self.desired_position.y, self.position.x, self.position.y ]
        #self.attitute_controller_yaw.setpoint= YawDes
        #dYaw    = self.attitute_controller_yaw(self.orientation.z)

        return RollDes, PitchDes, state

    def attitude_controller(self, RollDes, PitchDes, YawDes):

        self.attitute_controller_roll.setpoint=RollDes
        dRoll   = self.attitute_controller_roll(self.orientation.x)

        
        self.attitute_controller_pitch.setpoint=PitchDes 
        dPitch  = self.attitute_controller_pitch(self.orientation.y)


        self.attitute_controller_yaw.setpoint= YawDes
        dYaw    = self.attitute_controller_yaw(self.orientation.z)   
        
        state = [RollDes, PitchDes, YawDes, self.orientation.x, self.orientation.y, self.orientation.z]

                 
       
        return dRoll, dPitch, dYaw, state


    def rate_controller(self, dRollDes, dPitchDes, dYawDes):

        
        self.rate_controller_roll.setpoint=dRollDes
        ddRoll=self.rate_controller_roll(self.angularvelocity.x)
    
       
        self.rate_controller_pitch.setpoint=dPitchDes
        ddPitch=self.rate_controller_pitch(self.angularvelocity.y)
        
 

        self.rate_controller_yaw.setpoint=dYawDes
        ddYaw=self.rate_controller_yaw(self.angularvelocity.z)

 

        #log data
        state=[dRollDes, dPitchDes, dYawDes, self.angularvelocity.x, self.angularvelocity.y, self.angularvelocity.z, ddRoll, ddPitch, ddYaw]


       
        return ddRoll, ddPitch, ddYaw, state

    
    def __call__(self):
        #get time for logging
        try:
            current_time=rospy.get_time()
        except:
            rospy.logwarn("Could not get time. Check if roscore is running properly.")
            current_time=0

        if self.rate_controller_tuning_mode:

            #If the tuning mode is on, the attitude controller is disabled and the manual inputs are given directly to the rate controller.

            RollRate, PitchRate, YawRate = self.tuning_inputs

            throttle, throttle_state=self.altitude_controller()

            ddRoll, ddPitch, ddYaw, rate_state  = self.rate_controller(RollRate, PitchRate, YawRate)

            motor0, motor1, motor2, motor3= self.mixing_algorithm(throttle, ddRoll, ddPitch, ddYaw)

            #state =[self.ThrottleDes.z, self.position.z, throttle, 0, 0, 0, self.orientation.x, self.orientation.y, self.orientation.z, 0,0,0, RollRate, PitchRate, YawRate, self.angularvelocity.x, self.angularvelocity.y, self.angularvelocity.z, ddRoll, ddPitch, ddYaw, 0,0,0, current_time ]
            #self.log.append(state)

            return [motor0, motor1, motor2, motor3]
        


        throttle, throttle_state=self.altitude_controller()


        RollDes, PitchDes, position_state=self.position_controller()

        dRollDes, dPitchDes, dYawDes, attitude_state = self.attitude_controller(RollDes, PitchDes, self.YawDes)  #TODO: Add Yaw Control from Trajectory Messages. 
       
        ddRoll, ddPitch, ddYaw, rate_state  = self.rate_controller(dRollDes, dPitchDes, dYawDes)
        
        motor0, motor1, motor2, motor3= self.mixing_algorithm(throttle, ddRoll, ddPitch, ddYaw)

        
        state= throttle_state + position_state+  attitude_state + rate_state + [current_time]
      

        self.log.append( state )

    

        return [motor0, motor1, motor2, motor3]


    def mixing_algorithm(self, throttle , delta_rpm_roll, delta_rpm_pitch, delta_rpm_yaw):
             

        motor0 = throttle - delta_rpm_roll - delta_rpm_pitch + delta_rpm_yaw
        motor1 = throttle + delta_rpm_roll - delta_rpm_pitch - delta_rpm_yaw
        motor2 = throttle + delta_rpm_roll + delta_rpm_pitch + delta_rpm_yaw
        motor3 = throttle - delta_rpm_roll + delta_rpm_pitch - delta_rpm_yaw
        
        

        
        return motor0, motor1, motor2, motor3
 
     
    def reconfigure_cb(self, config, dummy):
        # Fill in local variables with values received from dynamic reconfigure
        # clients (typically the GUI).
        self.override_default_pid_vals=config["override_default_pid_vals"]
        if self.override_default_pid_vals:

            throttle_p      =config["throttle_p"]
            throttle_i      =config["throttle_i"]
            throttle_d      =config["throttle_d"]

            position_x_p      =config["position_x_p"]
            position_x_i      =config["position_x_i"]
            position_x_d      =config["position_x_d"]

            position_y_p      =config["position_y_p"]
            position_y_i      =config["position_y_i"]
            position_y_d      =config["position_y_d"]


            attitude_roll_p =config["attitude_roll_p"]
            attitude_roll_i =config["attitude_roll_i"]
            attitude_roll_d =config["attitude_roll_d"]

            attitude_pitch_p=config["attitude_pitch_p"]
            attitude_pitch_i=config["attitude_pitch_i"]
            attitude_pitch_d=config["attitude_pitch_d"]

            attitude_yaw_p  =config["attitude_yaw_p"]
            attitude_yaw_i  =config["attitude_yaw_i"]
            attitude_yaw_d  =config["attitude_yaw_d"]

            rate_roll_p     =config["rate_roll_p"]
            rate_roll_i     =config["rate_roll_i"]
            rate_roll_d     =config["rate_roll_d"]

            rate_pitch_p    =config["rate_pitch_p"]
            rate_pitch_i    =config["rate_pitch_i"]
            rate_pitch_d    =config["rate_pitch_d"]

            rate_yaw_p      =config["rate_yaw_p"]
            rate_yaw_i      =config["rate_yaw_i"]
            rate_yaw_d      =config["rate_yaw_d"]
        


            altitude_gain=rospy.search_param("altitude_gain")
            rospy.set_param(altitude_gain+"/P", throttle_p)
            rospy.set_param(altitude_gain+"/I", throttle_i)
            rospy.set_param(altitude_gain+"/D", throttle_d)

            position_x_gain=rospy.search_param("position_x_gain")
            rospy.set_param(position_x_gain+"/P", position_x_p)
            rospy.set_param(position_x_gain+"/I", position_x_i)
            rospy.set_param(position_x_gain+"/D", position_x_d)

            position_y_gain=rospy.search_param("position_y_gain")
            rospy.set_param(position_y_gain+"/P", position_y_p)
            rospy.set_param(position_y_gain+"/I", position_y_i)
            rospy.set_param(position_y_gain+"/D", position_y_d)


            attitude_roll_gain=rospy.search_param("attitude_roll_gain")
            rospy.set_param(attitude_roll_gain+"/P", attitude_roll_p)
            rospy.set_param(attitude_roll_gain+"/I", attitude_roll_i)
            rospy.set_param(attitude_roll_gain+"/D", attitude_roll_d)



            attitude_pitch_gain=rospy.search_param("attitude_pitch_gain")
            rospy.set_param(attitude_pitch_gain+"/P", attitude_pitch_p)
            rospy.set_param(attitude_pitch_gain+"/I", attitude_pitch_i)
            rospy.set_param(attitude_pitch_gain+"/D", attitude_pitch_d)


            attitude_yaw_gain=rospy.search_param("attitude_yaw_gain")
            rospy.set_param(attitude_yaw_gain+"/P", attitude_yaw_p)
            rospy.set_param(attitude_yaw_gain+"/I", attitude_yaw_i)
            rospy.set_param(attitude_yaw_gain+"/D", attitude_yaw_d)


            angular_rate_roll_gain=rospy.search_param("angular_rate_roll_gain")
            rospy.set_param(angular_rate_roll_gain+"/P", rate_roll_p)
            rospy.set_param(angular_rate_roll_gain+"/I", rate_roll_i)
            rospy.set_param(angular_rate_roll_gain+"/D", rate_roll_d)



            angular_rate_pitch_gain=rospy.search_param("angular_rate_pitch_gain")
            rospy.set_param(angular_rate_pitch_gain+"/P", rate_pitch_p)
            rospy.set_param(angular_rate_pitch_gain+"/I", rate_pitch_i)
            rospy.set_param(angular_rate_pitch_gain+"/D", rate_pitch_d)


            angular_rate_yaw_gain=rospy.search_param("angular_rate_yaw_gain")
            rospy.set_param(angular_rate_yaw_gain+"/P", rate_yaw_p)
            rospy.set_param(angular_rate_yaw_gain+"/I", rate_yaw_i)
            rospy.set_param(angular_rate_yaw_gain+"/D", rate_yaw_d)
            self.initialize_pid()
       
        
        self.rate_controller_tuning_mode = config["rate_controller_tuning_mode"]

        if self.rate_controller_tuning_mode:
            roll_rate=config["roll_rate"]
            pitch_rate=config["pitch_rate"]
            yaw_rate=config["yaw_rate"]
            self.tuning_inputs=[roll_rate,pitch_rate,yaw_rate]
        
       
        
        return config
     
    

class PubMotorSpeed():


    def __init__(self,rotorvelocities):
        """Read in parameters."""
        self.PubtoTopic="command/motor_speed"

        self.enable = True
       
        self.pub = rospy.Publisher(self.PubtoTopic, Actuators, queue_size=10)
        
        
        self.msg=Actuators()
        self.msg.angular_velocities=rotorvelocities
        
       
        
       
        if self.enable:
            self.start()
        else:
            self.stop()

        # Create a timer to go to a callback at a specified interval.
        self.timer_cb(rotorvelocities)

    def start(self):
        """Turn on publisher."""
        self.pub = rospy.Publisher(self.PubtoTopic, Actuators, queue_size=10)

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def timer_cb(self, rotorvelocities ):
        """Call at a specified interval to publish message."""

        self.msg.angular_velocities=rotorvelocities
        # Publish our custom message.
        self.pub.publish(self.msg)
        


class PubEulerAngles():


    def __init__(self,data):
        """Read in parameters."""
        self.PubtoTopic="orientation_euler_angles"

        self.enable = True
       
        self.pub = rospy.Publisher(self.PubtoTopic, Point, queue_size=10)
        
        
        self.msg=Point()
        self.msg.x, self.msg.y, self.msg.z = data

        
       
        
       
        if self.enable:
            self.start()
        else:
            self.stop()

        # Create a timer to go to a callback at a specified interval.
        self.timer_cb(data)

    def start(self):
        """Turn on publisher."""
        self.pub = rospy.Publisher(self.PubtoTopic, Point, queue_size=10)

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def timer_cb(self, data ):
        """Call at a specified interval to publish message."""

        self.msg.x, self.msg.y, self.msg.z = data
        # Publish our custom message.
        self.pub.publish(self.msg)



def data_handler(data):
    #read the data into 1d arrays. 
    
    data=np.array(data)
    
    data = np.where(data == NaN, 0.0, data)
    
    #ThrottleDes, Acceleration_z, Throttle, RollDes, PitchDes, YawDes, orientation_x, orientation_y, orientation_z, int_roll, int_pitch, int_yaw, dRollDes, dPitchDes, dYawDes, angularvelocity_x, angularvelocity_y, angularvelocity_z, ddRoll, ddPitch, ddYaw, rate_controller_roll_integral, rate_controller_pitch_integral,rate_controller_yaw_integral,  time = np.transpose(data)

    header="ThrottleDes_z; position_z; throttle ; desired_position_x; desired_position_y; position_x; position_y ;RollDes; PitchDes; orientation_x; orientation_y; dRollDes; dPitchDes; YawRateDes; angularvelocity_x; angularvelocity_y; angularvelocity_z; ddRoll; ddPitch; ddYaw; current_time"

    #define the directory to save the data. 
    filename =rospy.get_namespace()[1:-1]+"_positioncontroller_log"
    dir=os.path.expanduser("~")+"/"+ filename

    #create the directory if it doesn't exist.
    if not os.path.exists(dir):
        os.makedirs(dir)
        rospy.loginfo("%s folder not found.\nCreating %s file for the output.", filename , dir)
    
    
    np.savetxt(dir+"/Data.csv",data, delimiter=";", header=header, fmt='%.18e'   )

    rospy.loginfo("Successfully created log files at:  %s", str(dir))

    
    
    
# Main function.
if __name__ == "__main__":
    rospy.init_node("pid_position_controller")
    rospy.loginfo("pid_position_controller node started.")

    #initialize subscriber objects
    SubTraject=SubTrajectory()
    SubIMU_=SubIMU()
    OdomSub=SubOdometry()

    #initate the controller object
    controller=Controller(SubTraject,SubIMU_,OdomSub)

    #publish the first message
    PubMotor=PubMotorSpeed((controller()))


    PubAngle=PubEulerAngles([0,0,0])


    while not rospy.is_shutdown():

        #refresh the values from the subscribers
        SubTraject.start()
        SubIMU_.start()
        OdomSub.start()

        #send the readings to the control object
        controller.update(SubTraject ,SubIMU_,OdomSub)

        #calculate rotor velocities from control object
        RotorVels=controller()

        #Publish Euler Angles of the drone. This can be used for PID Tuning via rqt_plot tool.
        
        PubAngle.timer_cb([controller.orientation.x, controller.orientation.y, controller.orientation.z])

        #publish the rotor velocities
        PubMotor.timer_cb(RotorVels)
        
        try:
            #The sleep time is small since the frequency is controller by the step size of Gazebo
            rospy.sleep(0.0001)
        except ROSInterruptException:
            
            rospy.logwarn("ROS Interrupt Exception: A ROS shutdown was requested. Exiting main loop.\nPlease wait while your MiniQuad Data file is created. ")
            break
      
            


    
    #save log data to csv file and create plots
    data_handler(controller.log)


    # terminate topic connections. 
    SubTraject.stop()
    SubIMU_.stop()
    OdomSub.stop()
    PubMotor.stop()

    rospy.loginfo("miniquad_controller node exiting")
    

