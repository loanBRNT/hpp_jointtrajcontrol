import rclpy
from control_msgs.srv import QueryTrajectoryState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from rclpy.node import Node
from rclpy.action import ActionClient

import sys
from math import sqrt

def getRobotState():
    rclpy.init()
    node = Node('temp_client')
    client = node.create_client(QueryTrajectoryState, '/joint_trajectory_controller/query_state')

    while not client.wait_for_service(timeout_sec=1.0):
        print('Waiting for service /joint_trajectory_controller/query_state...')

    request = QueryTrajectoryState.Request()

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()
    node.destroy_node()
    rclpy.shutdown()
    return response

'''
Send a trajectory directly to the controller. Do not wait for execution feedback.
The functiion create and destroy a temp ROS2 node. Could be ressource intensive, fever the use of the sending_hpp node.
'''
def sendingTraj(ps, pathID, order, arm_id, inv=False):
    rclpy.init()
    node = Node('temp_client')
    client = ActionClient(node, FollowJointTrajectory, 
                                   '/joint_trajectory_controller/follow_joint_trajectory')

    if order not in [0,1,2]:
        print(f"ERROR : Bad derivative indice {order}.\n Must be 0,1 or 2")
        return False
    try:
        waypoints, times = ps.getWaypoints(pathID)
    except:
        print(f"ERROR : Failed to find a path with the id : {pathID}")
        return False

    trajectory_msg = FollowJointTrajectory.Goal()
    trajectory_msg.trajectory = generateMessage(ps,waypoints,times,order, pathID, arm_id, inv=inv)


    client.wait_for_server()
    goal_handle = client.send_goal_async(trajectory_msg)
    print("INFO : Trajectory sended")
    node.destroy_node()
    rclpy.shutdown()
    return True

'''
Generate a JointTrajectory based on waypoints and times
'''
def generateMessage(ps, waypoints, times, order, pathID, arm_id, no_grip=True, inv=False):
    trajectory = JointTrajectory()
    trajectory.points = []

    N = len(waypoints)-1
    t_final = times[N]

    if inv:
        r = range(N,-1,-1)
    else:
        r = range(0, len(waypoints))

    print(len(waypoints))
    print(N)
    print(r)

    for i in r:
        sys.stdout.flush()
        wp = waypoints[i]
        t = times[i]
        point = JointTrajectoryPoint()
        point.positions = extractRobotConfig(wp, no_grip, grip=wp[7])
        if order == 0:
            v = [0.0 for e in range(len(point.positions))]
            a = [0.0 for e in range(len(point.positions))]
        else:
            v = ps.derivativeAtParam(pathID, 1, t)
            if order == 2:
                a = ps.derivativeAtParam(pathID, 2, t)
            else:
                a = [0.0 for e in range(len(point.positions))]
        point.velocities = extractRobotConfig(v, no_grip)
        point.accelerations = extractRobotConfig(a, no_grip)

        t_traj = t if not inv else (t_final - t)
        
        point.time_from_start = Duration(sec=int(t_traj), nanosec=int((t_traj - int(t_traj)) * 1e9))

        trajectory.points.append(point)
        print(point)

    trajectory.joint_names = [f'{arm_id}_joint1',f'{arm_id}_joint2',f'{arm_id}_joint3',f'{arm_id}_joint4',f'{arm_id}_joint5',f'{arm_id}_joint6',f'{arm_id}_joint7']
    if not no_grip:
        trajectory.joint_names.append(f'{arm_id}_finger_joint1')

    return trajectory

# ==========================================================
# Following function need to be modified to be more generalist

def extractRobotConfig(q, no_grip, grip=0.0): 
    # Le param grip permet de passer la valeur pour le joint gripper. 
    # En position on envoie la valeur orginale.
    # En vitesse 0, car le gripper est statique.

    q_ros2 = q[0:7]
    if not no_grip:
        q_ros2.append(grip)

    # tri selon les noms de joint
    # for q_name in desired_joint_names:
    #    if q_name in self.gripper_value:
    #        q_ros2.append(self.gripper_value[q_name])
    #        continue
    #    for j in range(len(self.hpp_joint_names)):
    #        if q_name == self.hpp_joint_names[j]:
    #            q_ros2.append(q[j])
    #            break

    return q_ros2

# ==============================================================