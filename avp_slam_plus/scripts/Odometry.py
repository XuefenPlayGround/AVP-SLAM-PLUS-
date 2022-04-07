#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from math import sin, cos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

gazebo_c1 = 0.001
gazebo_c2 = 0.001
gazebo_c3 = 0.001
gazebo_c4 = 0.001

odom_c1 = 0.01
odom_c2 = 0.01
odom_c3 = 0.01
odom_c4 = 0.01
id = 0
pre_id = -1

previous_pose = np.array([0, 0, 0])

def wrap2pi(yaw):
    if yaw < -np.pi:
        yaw += 2*np.pi
    elif yaw > np.pi:
        yaw -= 2*np.pi
    return yaw

def odom_callback(data):
    global vertex_pub
    global edge_pub
    # global cmd_pub
    global first_vertex
    v = data.linear.x
    w = data.angular.z

    ######################################################################
    #################### calculate vel cmd for gazebo ####################
    ######################################################################

    gazebo_vel_cmd = Twist()
    g_R = np.array([[(gazebo_c1* abs(v)+gazebo_c2*abs(w))**2, 0], [0, (gazebo_c3* abs(v)+gazebo_c4*abs(w))**2]])
    g_rand = np.random.multivariate_normal([v, w], g_R)
    v_actual = g_rand[0]
    w_actual = g_rand[1]
    # print("actual: ", v_actual, w_actual)
    gazebo_vel_cmd.linear.x = v_actual
    gazebo_vel_cmd.angular.z = w_actual
    # cmd_pub.publish(data)
    # cmd_pub.publish(gazebo_vel_cmd)
    ######################################################################
    ################## ready to publish as noisy input ###################
    ######################################################################
    global previous_pose
    global id
    global pre_id
    global pre_time
    now = rospy.get_time()
    
    # delta_t = dt = now - pre_time
    delta_t = dt = 0.01
    theta = previous_pose[2]

    
    o_R = np.array([[(odom_c1* abs(v)+odom_c2*abs(w))**2, 0], [0, (odom_c3* abs(v)+odom_c4*abs(w))**2]]) + 1e-10
    o_rand = np.random.multivariate_normal([v, w], o_R)
    v_odom = o_rand[0]
    w_odom = o_rand[1]

    Vt = np.array([[ (-sin(theta)+sin(theta+w_odom*dt))/w_odom,  v_odom*(sin(theta)-sin(theta+w_odom*dt))/(w_odom**2) + (v_odom*cos(theta+w_odom*dt)*dt)/w_odom ],
                [ (cos(theta)-cos(theta+w_odom*dt))/w_odom,  -v_odom*(cos(theta)-cos(theta+w_odom*dt))/(w_odom**2) + (v_odom*sin(theta+w_odom*dt)*dt)/w_odom ],
                [0, dt]])

    cov = np.matmul(np.matmul(Vt, o_R), Vt.T)

    delta_x = v_odom*delta_t*np.cos(previous_pose[2] + w_odom*delta_t)
    delta_y = v_odom*delta_t*np.sin(previous_pose[2] + w_odom*delta_t)
    delta_theta = -w_odom * delta_t
    delta_pose = np.array([delta_x, delta_y, delta_theta])
    new_pose = previous_pose + delta_pose
    new_pose[2] = wrap2pi(new_pose[2])

    vertex_output = "VERTEX_SE2" + " " +  str(now) + " " + str(new_pose[0]) + " " + str(new_pose[1]) + " " + str(new_pose[2])
    edge_output = "EDGE_SE2" + " " + str(pre_time) + " " + str(now) + " " + str(v_odom*dt) + " " + str(0) + " " + str(w_odom*dt) \
        + " " + str(cov[0, 0]) + " " + str(cov[0, 1]) + " " + str(cov[0, 2]) + " " + str(cov[1, 1]) + " " + str(cov[1, 2]) + " " + str(cov[2, 2])
    # print(vertex_output)
    # print(edge_output)
    # print("Pose:", new_pose)
    # print("Delta", delta_pose)
    previous_pose = new_pose
    id += 1
    pre_id += 1
    if first_vertex == 0:
        # edge_pub.publish(edge_output)    
        vertex_pub.publish(vertex_output)
        first_vertex = 1
    else:    
        edge_pub.publish(edge_output)    
        vertex_pub.publish(vertex_output)    
    pre_time = now

if __name__ == '__main__':
    try:
        rospy.init_node('Odometry', anonymous=True)
        vertex_pub = rospy.Publisher('/vertex_odom', String, queue_size=10)
        edge_pub = rospy.Publisher('/edge_odom', String, queue_size=10)
        # cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pre_time = rospy.get_time()
        # subscribe ctrl_cmd
        rospy.Subscriber("/cmd_vel", Twist, odom_callback)
        first_vertex = 0
        rospy.spin()
    except rospy.ROSInterruptException:
        pass