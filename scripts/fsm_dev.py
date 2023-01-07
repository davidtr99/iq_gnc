#! /usr/bin/env python

import rospy
from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *
import numpy as np 
import math
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TwistStamped, PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry

# Constants
K_GROUP = 1.5
K_REPULSION = 2.0
K_FRICTION = 0.5
K_OBSTACLES = 1.5

VEL_MAX = 0.5
MIN_FORCE = 0.1

N_UAVS = 4

# FM variables
uavs_pose = [np.zeros(3) for _ in range(N_UAVS)]
uavs_velocities = [np.zeros(3) for _ in range(N_UAVS)]
uavs_forces = [{} for i in range(N_UAVS)]
forces_keys = ['f_group','f_repulsion','f_friction','f_obstacles']
obstacles = []
dt = 0.1

# Inicializamos las fuerzas:
for id in range(N_UAVS):
    for key in forces_keys:
        uavs_forces[id][key] = np.zeros(3)
    uavs_forces[id]['f_resultant'] = np.zeros(3)

# Velocidad a comandar a los UAVS:
out_vel_vector = [TwistStamped() for _ in range(N_UAVS)]

####################################################
# Force Model Functions
####################################################

# Calculo del centro de masas 
def center_of_mass(points):
    points = np.array(points)
    return np.mean(points, axis=0)

# Funcion que calcula la fuerza de grupo hacia el centro de masas
def group_forces(uavs, cm):
    for id, uav_pose in enumerate(uavs):
        dir = cm - uav_pose
        dir = dir / np.linalg.norm(dir)
        dist = np.linalg.norm(cm - uav_pose)
        if dist < 0.1:
            uavs_forces[id]['f_group'] = np.zeros(3)
        else:
            uavs_forces[id]['f_group'] = K_GROUP * dir

# Funcion que calcula la fuerza de repulsion al resto de los UAVs y al centro de masas
def repulsion_forces(uavs, cm):
    B = 2.0
    for id1, uav_pose1 in enumerate(uavs):
        f_repulsion = np.zeros(3)
        for id2, uav_pose2 in enumerate(uavs):
            if id1 == id2:
                continue
            
            dir = uav_pose2 - uav_pose1
            dir = dir / np.linalg.norm(dir)
            dist = np.linalg.norm(uav_pose2 - uav_pose1)
            f_repulsion += - dir / math.pow(dist,B)

        dir = cm - uav_pose1
        dir = dir / np.linalg.norm(dir)
        dist = np.linalg.norm(cm - uav_pose1)
        f_repulsion +=  - dir / math.pow(dist,B)
        uavs_forces[id1]['f_repulsion'] = K_REPULSION * f_repulsion

# Función que calcula la fuerza de fricción, constante y contraria a la velocidad
def friction_forces(uavs, uavs_velocities):
    for id, _ in enumerate(uavs):
        if np.linalg.norm(uavs_velocities[id]) < 0.05:
            uavs_forces[id]['f_friction'] = np.zeros(3)
        else:
            uavs_forces[id]['f_friction'] = - K_FRICTION * uavs_velocities[id]/np.linalg.norm(uavs_velocities[id])

# Función que calcula la fuerza de repulsión hacia los obstáculos
def obstacles_forces(uavs, obstacles):
    B = 4.0
    for id, uav_pose in enumerate(uavs):
        f_obstacles = np.zeros(3)
        for obstacle in obstacles:
            dir = obstacle - uav_pose
            dir = dir / np.linalg.norm(dir)
            dist = np.linalg.norm(obstacle - uav_pose)
            f_obstacles += - dir / math.pow(dist,B)
        uavs_forces[id]['f_obstacles'] = K_OBSTACLES * f_obstacles

# Función que calcula la fuerza resultante a aplicar a cada UAV
def resultant_forces(uavs_forces, forces_keys):
    for id, uav_forces in enumerate(uavs_forces):
        f_resultant = np.zeros(3)
        for key in forces_keys:
            if key != 'f_friction':
                f_resultant += uav_forces[key]
        if np.linalg.norm(f_resultant) < MIN_FORCE:
            f_resultant = np.zeros(3)
        
        f_resultant += uav_forces['f_friction']
        uavs_forces[id]['f_resultant'] = f_resultant

# Aplicamos las fuerzas para obtener las velocidades
def apply_forces(uavs, uavs_forces, dt):
    for id, uav_pose in enumerate(uavs):
        uavs_velocities[id] = uavs_velocities[id] + uavs_forces[id]['f_resultant'] * dt
        
        if np.linalg.norm(uavs_velocities[id]) > VEL_MAX:
            uavs_velocities[id] = uavs_velocities[id]/np.linalg.norm(uavs_velocities[id]) * VEL_MAX
        
        out_vel_vector[id].twist.linear.x = uavs_velocities[id][0]
        out_vel_vector[id].twist.linear.y = uavs_velocities[id][1]
        out_vel_vector[id].twist.linear.z = uavs_velocities[id][2] 


####################################################
# Callbacks
####################################################

flag_pose1 = False
def pose1_cb(msg):
    global flag_pose1
    flag_pose1 = True
    uavs_pose[0][0] = msg.pose.pose.position.x
    uavs_pose[0][1] = msg.pose.pose.position.y
    uavs_pose[0][2] = msg.pose.pose.position.z

flag_pose2 = False
def pose2_cb(msg):
    global flag_pose2
    flag_pose2 = True
    uavs_pose[1][0] = msg.pose.pose.position.x + 5
    uavs_pose[1][1] = msg.pose.pose.position.y
    uavs_pose[1][2] = msg.pose.pose.position.z

flag_pose3 = False
def pose3_cb( msg):
    global flag_pose3
    flag_pose3 = True
    uavs_pose[2][0] = msg.pose.pose.position.x + 5
    uavs_pose[2][1] = msg.pose.pose.position.y + 5
    uavs_pose[2][2] = msg.pose.pose.position.z

flag_pose4 = False
def pose4_cb(msg):
    global flag_pose4
    flag_pose4 = True
    uavs_pose[3][0] = msg.pose.pose.position.x
    uavs_pose[3][1] = msg.pose.pose.position.y +5
    uavs_pose[3][2] = msg.pose.pose.position.z

####################################################
# Main Program
####################################################

def init_ros():
    global vel1_pub, vel2_pub, vel3_pub, vel4_pub

    rospy.init_node("drone_controller", anonymous=True)

    # Publishers
    vel1_pub = rospy.Publisher(
        name="drone1/mavros/setpoint_velocity/cmd_vel",
        data_class=TwistStamped,
        queue_size=10,
    )
    vel2_pub = rospy.Publisher(
        name="drone2/mavros/setpoint_velocity/cmd_vel",
        data_class=TwistStamped,
        queue_size=10,
    )
    vel3_pub = rospy.Publisher(
        name="drone3/mavros/setpoint_velocity/cmd_vel",
        data_class=TwistStamped,
        queue_size=10,
    )
    vel4_pub = rospy.Publisher(
        name="drone4/mavros/setpoint_velocity/cmd_vel",
        data_class=TwistStamped,
        queue_size=10,
    )

    # Subscribers
    currentPos1 = rospy.Subscriber(
        name="drone1/mavros/global_position/local",
        data_class=Odometry,
        queue_size=10,
        callback=pose1_cb,
    )
    currentPos2 = rospy.Subscriber(
        name="drone2/mavros/global_position/local",
        data_class=Odometry,
        queue_size=10,
        callback=pose2_cb,
    )
    currentPos3 = rospy.Subscriber(
        name="drone3/mavros/global_position/local",
        data_class=Odometry,
        queue_size=10,
        callback=pose3_cb,
    )
    currentPos4 = rospy.Subscriber(
        name="drone4/mavros/global_position/local",
        data_class=Odometry,
        queue_size=10,
        callback=pose4_cb,
    )



def main():

    # Inicializamos ROS
    init_ros()
    rate = rospy.Rate(1.0/dt)

    # Wait until all the callbacks are called
    while not (flag_pose1 and flag_pose2 and flag_pose3 and flag_pose4):
        rate.sleep()

    ####
    # [2] DESCOMENTAR ESTO PARA QUE LOS DRONES SE MUEVAN EN UN CUADRADO
    # cm = center_of_mass(uavs_pose)
    # i = 0
    ####

    ####
    # [3] DESCOMENTAR ESTO PARA QUE LOS DRONES VAYAN A UN TARGET
    # cm = np.array([0.0,0.0,5.0])
    # print("cm: ", cm)
    ####

    ####
    # [4] DESCOMENTAR ESTO PARA EVITACION DE OBSTACULO
    # Columna vertical de puntos en el centro
    # obstacles = np.array([[0.0,9.5,0.0],[0.0,9.5,1.0],[0.0,9.5,2.0],[0.0,9.5,3.0],[0.0,9.5,4.0],[0.0,9.5,5.0],[0.0,9.5,6.0],[0.0,9.5,7.0],[0.0,9.5,8.0],[0.0,9.5,9.0],[0.0,9.5,10.0],])
    ####


    while not rospy.is_shutdown():
        
        ####
        # [1] DESCOMENTAR ESTO PARA QUE LOS DRONES SE ESTABILICEN EN SU CENTRO DE MASAS
        cm = center_of_mass(uavs_pose)
        ####

        ####
        # [2] DESCOMENTAR ESTO PARA QUE LOS DRONES SE MUEVAN EN UN CUADRADO
        # if i < 100:
        #     cm += np.array([0.1,0.00,0.00])
        # elif i < 200:
        #     cm += np.array([0.00,0.1,0.00])
        # elif i < 300:
        #     cm += np.array([-0.1,0.00,0.00])
        # elif i < 400:
        #     cm += np.array([0.00,-0.1,0.00])
        # else:
        #     i = 0
        # i += 1
        ####

        ####
        # [4] DESCOMENTAR ESTO PARA EVITACION DE OBSTACULO
        # cm += np.array([0.0,-0.1,0.0])
        ####

        # Calculo de fuerzas
        group_forces(uavs_pose, cm)
        repulsion_forces(uavs_pose, cm)
        friction_forces(uavs_pose, uavs_velocities)
        obstacles_forces(uavs_pose, obstacles)
        resultant_forces(uavs_forces, forces_keys)

        # Aplicamos las fuerzas
        apply_forces(uavs_pose, uavs_forces, dt)

        # Publicamos  velocidades
        vel1_pub.publish(out_vel_vector[0])
        vel2_pub.publish(out_vel_vector[1])
        vel3_pub.publish(out_vel_vector[2])
        vel4_pub.publish(out_vel_vector[3])
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
