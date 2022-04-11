#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import numpy as np
from pid_controller import *
# commit try 1


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
        
        self.hovering_throttle=874


    def altitude_controller(self):
        self.throttle_controller.setpoint= self.ThrottleDes.z
        throttle = self.throttle_controller(self.position.z) + self.hovering_throttle
        state= [self.ThrottleDes.z, self.position.z, throttle ]
        print("altitude_desired: %f, position_z: %f, throttle: %f".format(self.ThrottleDes.z, self.position.z, throttle))

        dt, bool_, now, last_time= self.throttle_controller.get_time_data()
        print("dt, bool_, now, last_time",dt, bool_, now, last_time  )
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
    

