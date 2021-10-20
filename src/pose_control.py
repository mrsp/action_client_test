#! /usr/bin/env python
import rospy
import actionlib
import copy
import whole_body_ik_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
CoM = False
LLeg = False
RLeg = False
odom = False
joints = False
LLeg_odom = Odometry()
RLeg_odom = Odometry()
CoM_odom = Odometry()
base_odom = Odometry()
joint_state = JointState()


def JointStatecallback(data):
    global joint_state
    joint_state = data
    global joints 
    joints = True

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

def OdomCallback(data):
    global base_odom
    base_odom = data
    global odom
    odom = True 

def action_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/humanoid/whole_body_control', whole_body_ik_msgs.msg.HumanoidAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("WBC Server is UP")
    # Creates a goal to send to the action server.
    goal = whole_body_ik_msgs.msg.HumanoidGoal()

    goal.odom = base_odom
    goal.joint_state = joint_state

    goal.dt = 0.01 #Hardware Control Cycle of NAO

    #Do Not Change
    dof_gain = 0.6
    dof_weight = 5.0e-05
    goal.Joints = []
    tmp_dof_task = whole_body_ik_msgs.msg.DOFTask()
    tmp_dof_task.weight = dof_weight
    tmp_dof_task.gain = dof_gain
    #HEAD
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "HeadYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "HeadPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    

    
    #LEFT LEG
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "LHipYawPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "LHipRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.3976
    tmp_dof_task.name = "LHipPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle =  0.85
    tmp_dof_task.name = "LKneePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.4427
    tmp_dof_task.name = "LAnklePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.009
    tmp_dof_task.name = "LAnkleRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))


    #RIGHT LEG
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "RHipYawPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.0
    tmp_dof_task.name = "RHipRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.3976
    tmp_dof_task.name = "RHipPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle =  0.85
    tmp_dof_task.name = "RKneePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.4427
    tmp_dof_task.name = "RAnklePitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.009
    tmp_dof_task.name = "RAnkleRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
  
    #LEFT ARM
    tmp_dof_task.desired_angle =  1.5
    tmp_dof_task.name = "LShoulderPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.15
    tmp_dof_task.name = "LShoulderRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0
    tmp_dof_task.name = "LElbowYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle =  -0.0349066
    tmp_dof_task.name = "LElbowRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -1.5
    tmp_dof_task.name = "LWristYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0
    tmp_dof_task.name = "LHand"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))

    #RIGHT ARM
    tmp_dof_task.desired_angle =  1.5
    tmp_dof_task.name = "RShoulderPitch"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = -0.15
    tmp_dof_task.name = "RShoulderRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle =  0
    tmp_dof_task.name = "RElbowYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0.0349066
    tmp_dof_task.name = "RElbowRoll"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 1.5
    tmp_dof_task.name = "RWristYaw"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))
    tmp_dof_task.desired_angle = 0
    tmp_dof_task.name = "RHand"
    goal.Joints.append(copy.deepcopy(tmp_dof_task))



    #Set Linear Task for CoM
    goal.CoM.linear_task.desired_position.x = CoM_odom.pose.pose.position.x
    goal.CoM.linear_task.desired_position.y = CoM_odom.pose.pose.position.y
    goal.CoM.linear_task.desired_position.z = CoM_odom.pose.pose.position.z
    goal.CoM.linear_task.desired_linear_velocity.x = CoM_odom.twist.twist.linear.x
    goal.CoM.linear_task.desired_linear_velocity.y = CoM_odom.twist.twist.linear.y
    goal.CoM.linear_task.desired_linear_velocity.z = CoM_odom.twist.twist.linear.z
    goal.CoM.linear_task.weight = 5.0e-1
    goal.CoM.linear_task.gain  = 0.35


    #Set Angular Task for Torso
    goal.Torso.angular_task.desired_angular_velocity.x = 0.0
    goal.Torso.angular_task.desired_angular_velocity.y = 0.0
    goal.Torso.angular_task.desired_angular_velocity.z = 0.0
    goal.Torso.angular_task.desired_orientation.x = 0.0
    goal.Torso.angular_task.desired_orientation.y = 0.0
    goal.Torso.angular_task.desired_orientation.z = 0.0
    goal.Torso.angular_task.desired_orientation.w = 1.0
    goal.Torso.angular_task.weight = 1.0e-3
    goal.Torso.angular_task.gain  = 0.2


    #Set Leg Task to keep their pose w.r.t ground
    goal.LLeg.linear_task.desired_position.x = LLeg_odom.pose.pose.position.x
    goal.LLeg.linear_task.desired_position.y = LLeg_odom.pose.pose.position.y
    goal.LLeg.linear_task.desired_position.z = LLeg_odom.pose.pose.position.z
    goal.LLeg.linear_task.desired_linear_velocity.x = 0.000
    goal.LLeg.linear_task.desired_linear_velocity.y = 0.000
    goal.LLeg.linear_task.desired_linear_velocity.z = 0.000
    goal.LLeg.linear_task.weight = 2.0
    goal.LLeg.linear_task.gain  = 0.35

    goal.LLeg.angular_task.desired_angular_velocity.x = 0.0
    goal.LLeg.angular_task.desired_angular_velocity.y = 0.0
    goal.LLeg.angular_task.desired_angular_velocity.z = 0.0
    goal.LLeg.angular_task.desired_orientation.x = LLeg_odom.pose.pose.orientation.x
    goal.LLeg.angular_task.desired_orientation.y = LLeg_odom.pose.pose.orientation.y
    goal.LLeg.angular_task.desired_orientation.z = LLeg_odom.pose.pose.orientation.z
    goal.LLeg.angular_task.desired_orientation.w = LLeg_odom.pose.pose.orientation.w
    goal.LLeg.angular_task.weight = 2.0
    goal.LLeg.angular_task.gain  = 0.2


    goal.RLeg.linear_task.desired_position.x = RLeg_odom.pose.pose.position.x
    goal.RLeg.linear_task.desired_position.y = RLeg_odom.pose.pose.position.y
    goal.RLeg.linear_task.desired_position.z = RLeg_odom.pose.pose.position.z
    goal.RLeg.linear_task.desired_linear_velocity.x = 0.000
    goal.RLeg.linear_task.desired_linear_velocity.y = 0.000
    goal.RLeg.linear_task.desired_linear_velocity.z = 0.000
    goal.RLeg.linear_task.weight = 2.0
    goal.RLeg.linear_task.gain  = 0.35

    goal.RLeg.angular_task.desired_orientation.x = RLeg_odom.pose.pose.orientation.x
    goal.RLeg.angular_task.desired_orientation.y = RLeg_odom.pose.pose.orientation.y
    goal.RLeg.angular_task.desired_orientation.z = RLeg_odom.pose.pose.orientation.z
    goal.RLeg.angular_task.desired_orientation.w = RLeg_odom.pose.pose.orientation.w
    goal.RLeg.angular_task.desired_angular_velocity.x = 0.0
    goal.RLeg.angular_task.desired_angular_velocity.y = 0.0
    goal.RLeg.angular_task.desired_angular_velocity.z = 0.0
    goal.RLeg.angular_task.weight = 2.0
    goal.RLeg.angular_task.gain  = 0.2




    ####################################################################
    ####################################################################
    #ARM  + Head CONTROL IS HERE
    goal.RHand.linear_task.desired_position.x = 0 #Fix this 
    goal.RHand.linear_task.desired_position.y = 0 #Fix this 
    goal.RHand.linear_task.desired_position.z = 0 #Fix this 
    goal.RHand.linear_task.desired_linear_velocity.x = 0.000 #Leave 0 here
    goal.RHand.linear_task.desired_linear_velocity.y = 0.000 #Leave 0 here
    goal.RHand.linear_task.desired_linear_velocity.z = 0.000 #Leave 0 here
    goal.RHand.linear_task.weight = -1 #Fix this 
    goal.RHand.linear_task.gain  = 0.35 #Fix this 


    goal.RHand.angular_task.desired_orientation.x = 0 #Fix this 
    goal.RHand.angular_task.desired_orientation.y = 0 #Fix this 
    goal.RHand.angular_task.desired_orientation.z = 0 #Fix this 
    goal.RHand.angular_task.desired_orientation.w = 0 #Fix this 
    goal.RHand.angular_task.desired_angular_velocity.x = 0.0 #Leave 0 here
    goal.RHand.angular_task.desired_angular_velocity.y = 0.0 #Leave 0 here
    goal.RHand.angular_task.desired_angular_velocity.z = 0.0 #Leave 0 here
    goal.RHand.angular_task.weight = -1 #Fix this 
    goal.RHand.angular_task.gain  = 0.2 #Fix this 

    goal.LHand.linear_task.desired_position.x = 0 #Fix this 
    goal.LHand.linear_task.desired_position.y = 0 #Fix this 
    goal.LHand.linear_task.desired_position.z = 0 #Fix this 
    goal.LHand.linear_task.desired_linear_velocity.x = 0.000 #Leave 0 here
    goal.LHand.linear_task.desired_linear_velocity.y = 0.000 #Leave 0 here
    goal.LHand.linear_task.desired_linear_velocity.z = 0.000 #Leave 0 here
    goal.LHand.linear_task.weight = -1 #Fix this 
    goal.LHand.linear_task.gain  = 0.35 #Fix this 


    goal.LHand.angular_task.desired_orientation.x = 0 #Fix this 
    goal.LHand.angular_task.desired_orientation.y = 0 #Fix this 
    goal.LHand.angular_task.desired_orientation.z = 0 #Fix this 
    goal.LHand.angular_task.desired_orientation.w = 0 #Fix this 
    goal.LHand.angular_task.desired_angular_velocity.x = 0.0 #Leave 0 here
    goal.LHand.angular_task.desired_angular_velocity.y = 0.0 #Leave 0 here
    goal.LHand.angular_task.desired_angular_velocity.z = 0.0 #Leave 0 here
    goal.LHand.angular_task.weight = -1 #Fix this 
    goal.LHand.angular_task.gain  = 0.2 #Fix this 


    #Translating the Head is not adviced
    # goal.Head.linear_task.desired_position.x = 0 #Fix this 
    # goal.Head.linear_task.desired_position.y = 0 #Fix this 
    # goal.Head.linear_task.desired_position.z = 0 #Fix this 
    # goal.Head.linear_task.desired_linear_velocity.x = 0.000 #Leave 0 here
    # goal.Head.linear_task.desired_linear_velocity.y = 0.000 #Leave 0 here
    # goal.Head.linear_task.desired_linear_velocity.z = 0.000 #Leave 0 here
    # goal.Head.linear_task.weight = 0 #Fix this 
    # goal.Head.linear_task.gain  = 0 #Fix this 


    goal.Head.angular_task.desired_orientation.x = 0 #Fix this 
    goal.Head.angular_task.desired_orientation.y = 0 #Fix this 
    goal.Head.angular_task.desired_orientation.z = 1 #Fix this 
    goal.Head.angular_task.desired_orientation.w = 0 #Fix this 
    goal.Head.angular_task.desired_angular_velocity.x = 0.0 #Leave 0 here
    goal.Head.angular_task.desired_angular_velocity.y = 0.0 #Leave 0 here
    goal.Head.angular_task.desired_angular_velocity.z = 0.0 #Leave 0 here
    goal.Head.angular_task.weight = 5.0e-4 #Fix this 
    goal.Head.angular_task.gain  = 0.2 #Fix this 
    ####################################################################
    ####################################################################

    # Sends the goal to the action server.
    client.send_goal(goal)
    print("Goal Sent")
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("Result received")
    # Prints out the result of executing the action
    return client.get_result()  

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('action_client_pose_task_test')
        rospy.Subscriber("/nao_raisim_ros/LLeg/odom", Odometry, LLegcallback)
        rospy.Subscriber("/nao_raisim_ros/RLeg/odom", Odometry, RLegcallback)
        rospy.Subscriber("/nao_raisim_ros/CoM", Odometry, CoMcallback)
        rospy.Subscriber("/nao_raisim_ros/odom", Odometry, OdomCallback)
        rospy.Subscriber("/nao_raisim_ros/joint_states", JointState, JointStatecallback)
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            print("Waiting for Data")
            rate.sleep()
            if(LLeg and RLeg and CoM and odom and joints):
                break

        print("Data Received")
        result = action_client()
        print("Result:", action_client)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
