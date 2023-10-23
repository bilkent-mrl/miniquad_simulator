#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# commit try 1

import rospy
import numpy as np
from numpy.core.numeric import NaN
from mav_msgs.msg import Actuators

from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, Point, Transform
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from rotors_control.cfg import velocity_commandConfig as ConfigType
from tf.transformations import euler_from_quaternion





    
class TrajectoryGenerator():
    def __init__(self):
        self.position=Vector3()
        self.velocity=Vector3()
        self.last_time=0

        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)
    def __call__(self, velocity):

        current_time=rospy.get_time()
    

        dt=current_time-self.last_time
        velocity=np.array([self.velocity.x,self.velocity.y, self.velocity.z])
        dPosition=velocity*dt
        self.position.x+=dPosition[0]
        self.position.y+=dPosition[1]
        # self.position.z+=dPosition[2]
        self.last_time=current_time
        return self.position

    def reconfigure_cb(self, config, dummy):
        # Fill in local variables with values received from dynamic reconfigure
        # clients (typically the GUI).
        self.velocity.x = config['vx']
        self.velocity.y = config['vy']
        self.position.z = config['altitude']
        
        return config




class PubTrajectory():

    def __init__(self):
        """Read in parameters."""
        self.PubtoTopic="command/trajectory"

        self.enable = True
       
        self.pub = rospy.Publisher(self.PubtoTopic, MultiDOFJointTrajectory, queue_size=10)
        
        
        self.msg=MultiDOFJointTrajectory()
        position=Vector3()
        
        self.data=MultiDOFJointTrajectoryPoint()
        print(self.data)
        
        transform=Transform()
        self.data.transforms.append(transform)
        self.data.transforms[0].translation=position
        # self.data.transforms[0].rotation=Quaternion(0,0,0,1)
    
        self.msg.points.append(self.data)
        

        
       
        if self.enable:
            self.start()
        else:
            self.stop()

        # Create a timer to go to a callback at a specified interval.
        self.timer_cb(Vector3())

    def start(self):
        """Turn on publisher."""
        self.pub = rospy.Publisher(self.PubtoTopic, MultiDOFJointTrajectory, queue_size=10)

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def timer_cb(self, position):
        """Call at a specified interval to publish message."""
                
        self.data.transforms[0].translation=position

        self.msg.points[0]=self.data
        
        # Publish our custom message.
        self.pub.publish(self.msg)
        





# def data_handler(data):
#     #read the data into 1d arrays. 
    
#     data=np.array(data)
    
#     data = np.where(data == NaN, 0.0, data)
    
#     #ThrottleDes, Acceleration_z, Throttle, RollDes, PitchDes, YawDes, orientation_x, orientation_y, orientation_z, int_roll, int_pitch, int_yaw, dRollDes, dPitchDes, dYawDes, angularvelocity_x, angularvelocity_y, angularvelocity_z, ddRoll, ddPitch, ddYaw, rate_controller_roll_integral, rate_controller_pitch_integral,rate_controller_yaw_integral,  time = np.transpose(data)

#     header="ThrottleDes_z; position_z; throttle ; desired_position_x; desired_position_y; position_x; position_y ;RollDes; PitchDes; orientation_x; orientation_y; dRollDes; dPitchDes; YawRateDes; angularvelocity_x; angularvelocity_y; angularvelocity_z; ddRoll; ddPitch; ddYaw; current_time"

#     #define the directory to save the data. 
#     filename =rospy.get_namespace()[1:-1]+"_positioncontroller_log"
#     dir=os.path.expanduser("~")+"/"+ filename

#     #create the directory if it doesn't exist.
#     if not os.path.exists(dir):
#         os.makedirs(dir)
#         rospy.loginfo("%s folder not found.\nCreating %s file for the output.", filename , dir)
    
    
#     np.savetxt(dir+"/Data.csv",data, delimiter=";", header=header, fmt='%.18e'   )

#     rospy.loginfo("Successfully created log files at:  %s", str(dir))

    
    
    
# Main function.
if __name__ == "__main__":
    rospy.init_node("velocity_term_generator")
    rospy.loginfo("velocity_term_generator node started.")
    



    position=Vector3()



    TrajGen=TrajectoryGenerator()

    #publish the trajectory message
    PubTraj=PubTrajectory()
    
    velocity=np.array([0,0,0])
    while not rospy.is_shutdown():


        
        position=TrajGen(velocity)
        #Publish Euler Angles of the drone. This can be used for PID Tuning via rqt_plot tool.
        

        PubTraj.timer_cb(position)

        
        
        try:
            #The sleep time is small since the frequency is controller by the step size of Gazebo
            rospy.sleep(0.0001)
        except ROSInterruptException:
            
            rospy.logwarn("ROS Interrupt Exception: A ROS shutdown was requested. Exiting main loop.\nPlease wait while your MiniQuad Data file is created. ")
            break
      
            


    
    #save log data to csv file and create plots


    # terminate topic connections. 


    rospy.loginfo("velocity_term_generator node exiting")
    

