#!/usr/bin/env python3
import rospy
import math
import numpy as np
import random

from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



from evry_project_plugins.srv import DistanceToFlag


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-5.0, max=5.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def PID_control(self, kp, ki, kd):
        #plaplapla
        self = []



def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")

    # Timing between robots
    rospy.sleep(3 * int(robot_name[-1]))

    # Strategy:
    # PID for flag, set speed & rotation for Obstacles
    
    # PID coefficients
    kp = 0.5
    ki = 0.1
    kd = 0.01
   
    # Initialize algorithm parameters
    vt = 0
    angle = 0
    d_des = 0
    err_old = 0
    angle_err_old = 0
    dt = 0.05       # delay for derivative in PID calculation line 
    
    # Initialize the robot speed
    robot.set_speed_angle(vt, angle)

    # get the flag ditance and pose:   We edited the DistanceToFlag service and code to obtain the pose 
    flag1 = [robot.getDistanceToFlag().flag_x, robot.getDistanceToFlag().flag_y]
    print(f"{robot_name} Flag coordinates = ", flag1[0] , " ", flag1[1])

    # start simulation loop
    while not rospy.is_shutdown():

        # Get the sensors redings
        sonar = robot.get_sonar()
        pose  = robot.get_robot_pose()
        dist = float(robot.getDistanceToFlag().distance)

        # Write here your strategy..

        # PID controller for speed
        err = dist-d_des
        vt = kp*err + ki*(err_old+err) + kd*(err-err_old)/dt

        # P controller for angle
        angle_diff = math.atan2(flag1[1]-pose[1], flag1[0]-pose[0])  # using poses of robot and flag
        angle_err  = angle_diff - pose[2]
        va = kp*angle_err +  ki*(angle_err_old+angle_err) +  kd*(angle_err-angle_err_old)/dt

        # display some parameters 
        print(f"{robot_name} sonar = ",sonar)
        print(f"{robot_name} flag = ",dist)
        print(f"{robot_name} diviation = ",angle,"radian")

    

        if sonar < 4.0 and dist > 0:  # Obstacles avoidance part            
            robot.set_speed_angle(2,0.3)
            rospy.sleep(3)


        # Stop robot at small distance
        if dist < 0.3:             
          vt = 0
          va = 0

        # store error values for next time step
        err_old = err
        angle_err_old = angle_err

        # Finishing by publishing the desired speed. 
        # DO NOT TOUCH.
        robot.set_speed_angle(vt,va)
        rospy.sleep(dt)



if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()