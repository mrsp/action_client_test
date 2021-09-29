#! /usr/bin/env python

import rospy
import actionlib
import copy
import lipm_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped



NF = 10
CoM = False
ZMP = False
LLeg = False
RLeg = False
LLeg_odom = Odometry()
RLeg_odom = Odometry()
CoM_odom = Odometry()
ZMP_odom = PointStamped()
def LLegcallback(data):
    global LLeg_odom
    LLeg_odom = data
    global LLeg 
    LLeg = True

def RLegcallback(data):
    global RLeg_odom
    RLeg_odom = data
    global RLeg
    RLeg = True


def CoMcallback(data):
    global CoM_odom
    CoM_odom = data
    global CoM 
    CoM = True

def ZMPcallback(data):
    global ZMP_odom
    ZMP_odom = data
    global ZMP
    ZMP = True


def action_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/lipm_motion/plan', lipm_msgs.msg.MotionPlanAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("Server is UP")


    # Creates a goal to send to the action server.
    goal = lipm_msgs.msg.MotionPlanGoal()
    temp =  lipm_msgs.msg.StepTarget()


    goal.lfoot.position.x = LLeg_odom.pose.pose.position.x
    goal.lfoot.position.y = LLeg_odom.pose.pose.position.y
    goal.lfoot.position.z = LLeg_odom.pose.pose.position.z
    goal.lfoot.orientation.x = LLeg_odom.pose.pose.orientation.x
    goal.lfoot.orientation.y = LLeg_odom.pose.pose.orientation.y
    goal.lfoot.orientation.z = LLeg_odom.pose.pose.orientation.z
    goal.lfoot.orientation.w = LLeg_odom.pose.pose.orientation.w
    
    goal.rfoot.position.x = RLeg_odom.pose.pose.position.x
    goal.rfoot.position.y = RLeg_odom.pose.pose.position.y
    goal.rfoot.position.z = RLeg_odom.pose.pose.position.z
    goal.rfoot.orientation.x = RLeg_odom.pose.pose.orientation.x
    goal.rfoot.orientation.y = RLeg_odom.pose.pose.orientation.y
    goal.rfoot.orientation.z = RLeg_odom.pose.pose.orientation.z
    goal.rfoot.orientation.w = RLeg_odom.pose.pose.orientation.w


    goal.CoM.pose.pose.position.x = CoM_odom.pose.pose.position.x #CoM_odom.pose.pose.position.x
    goal.CoM.pose.pose.position.y = CoM_odom.pose.pose.position.y
    goal.CoM.pose.pose.position.z = CoM_odom.pose.pose.position.z
    
    goal.COP.x =  ZMP_odom.point.x #CoM_odom.pose.pose.position.x
    goal.COP.y =  ZMP_odom.point.y #CoM_odom.pose.pose.position.y
    goal.COP.z = 0


    goal.CoM.twist.twist.linear.x = 0 #CoM_odom.twist.twist.linear.x
    goal.CoM.twist.twist.linear.y = 0 #CoM_odom.twist.twist.linear.y
    goal.CoM.twist.twist.linear.z = 0 #CoM_odom.twist.twist.linear.z



    goal.footsteps = []

    i=1
    while(i<NF):
        temp.leg = 1 #SWING LEG
        temp.pose.position.x = RLeg_odom.pose.pose.position.x + 0.04*i
        temp.pose.position.y = RLeg_odom.pose.pose.position.y
        temp.pose.position.z = RLeg_odom.pose.pose.position.z
        temp.pose.orientation.x = 0.0
        temp.pose.orientation.y = 0.0
        temp.pose.orientation.z = 0.0
        temp.pose.orientation.w = 1.0
        goal.footsteps.append(copy.deepcopy(temp))

        temp.leg = 0 #SWING LEG
        temp.pose.position.x = LLeg_odom.pose.pose.position.x + 0.04*i
        temp.pose.position.y = LLeg_odom.pose.pose.position.y
        temp.pose.position.z = LLeg_odom.pose.pose.position.z
        temp.pose.orientation.x = 0.0
        temp.pose.orientation.y = 0.0
        temp.pose.orientation.z = 0.0
        temp.pose.orientation.w = 1.0
        goal.footsteps.append(copy.deepcopy(temp))

        i=i+1



  
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("Goal Sent")

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("Result receivded")

    # Prints out the result of executing the action
    return client.get_result() 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('r')
        rospy.Subscriber("/nao_raisim_ros/LLeg/odom", Odometry, LLegcallback)
        rospy.Subscriber("/nao_raisim_ros/RLeg/odom", Odometry, RLegcallback)
        rospy.Subscriber("/nao_raisim_ros/CoM", Odometry, CoMcallback)
        rospy.Subscriber("/nao_raisim_ros/ZMP", PointStamped, ZMPcallback)

        rate = rospy.Rate(50) # 10hz
        while not rospy.is_shutdown():
            print("loop")
            rate.sleep()
            if(LLeg and RLeg and CoM and ZMP):
                break

        print("Out")

        result = action_client()
        print("Result:", action_client)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
