# Author : Loan BERNAT (l.bernat@sileane.com)

from hpp.corbaserver.manipulation import Robot
from hpp.corbaserver import loadServerPlugin, shrinkJointRange
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, CorbaClient, SecurityMargins, Constraints
from hpp.gepetto.manipulation import ViewerFactory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.srv import QueryTrajectoryState
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time

from ament_index_python.packages import get_package_share_directory
import os

# Example of commande
# ros2 topic pub /goalConfig sensor_msgs/msg/JointState "{name: ['fer_joint1','fer_joint2','fer_joint3','fer_joint4','fer_joint5','fer_joint6','fer_joint7','finger_joint1','finger_joint2'], position: [0.0, 0.5, 0.4, -1.0, 0.0, 1.0, 0.0, 1.0, 1.0]}" --once
# ros2 topic pub /goalConfig sensor_msgs/msg/JointState "{position: [0.0, 0.5, 0.4, -1.0, 0.0, 1.0, 0.0, 1.0, 1.0]}" --once

class HPPSimple(Node):
    '''
    This node allows to control a Robot with a trajectory generated using HPP throught the ros2_control JointTrajectoryController.
    Currently, you cannot control the gripper due to HPP assumption.

    To use this node, you need to publish a sensor_msgs/msg/JointState on the topic /goalConfig with the joint_names and joint values.
    You need to respect the HPP order for configuration [robot, robot_finger1, robot_finger2, obj1, obj2 ..]
    IMPORTANT : For both grippers joint (Left / Right), give a name containing "finger".
    '''
    # These variables are changed dynamically in the code.
    q_goal = None
    q_init = None
    hpp_joint_names = None
    gripper_value = None

    SUPPORTED_ROBOTS = ['fer','fr3']

    def __init__(self):
        super().__init__('hpp_node')

        self.declare_parameter('arm_id', 'fer')
        self.arm_id = self.get_parameter('arm_id').value

        self.declare_parameter('load_grip', False)
        self.no_grip = not self.get_parameter('load_grip').value

        if self.arm_id not in self.SUPPORTED_ROBOTS:
            raise RuntimeError(f"ERROR : Bad argument.\nRobot {self.arm_id} is not supported. Avalaible robots {self.SUPPORTED_ROBOTS}")

        p_u = os.path.join(get_package_share_directory("hpp_control"),"urdf",f"{self.arm_id}.urdf")
        p_s = os.path.join(get_package_share_directory("hpp_control"),"srdf",f"{self.arm_id}.srdf")

        with open(p_u, "r") as f:
            urdf_string = f.read()

        with open(p_s, "r") as f:
            srdf_string = f.read()

        Robot.urdfString = urdf_string
        Robot.srdfString = srdf_string 

        defaultContext = "corbaserver"
        loadServerPlugin(defaultContext, "manipulation-corba.so")

        # Send the trajectory to the controller
        self.controller = ActionClient(self, FollowJointTrajectory, 
                                   '/joint_trajectory_controller/follow_joint_trajectory')
        
        # Receive the goal config by the user
        self.subscriber = self.create_subscription(JointState, '/goalConfig', self.askQinit, 2)

        # Receive the current config of the robot
        self.state_service = self.create_client(QueryTrajectoryState, '/joint_trajectory_controller/query_state')

        


    def setupHPP(self):
        newProblem()

        self.robot = Robot("robot", "pandas", rootJointType="anchor")
        self.robot.setRootJointPosition("pandas",[0,0,0.4,0,0,0,1]) # A changer selon l'env
        shrinkJointRange(self.robot, [f'pandas/{self.arm_id}_joint{i}' for i in range(1,8)],0.95)
        self.ps = ProblemSolver(self.robot)

        # self.vf = ViewerFactory(self.ps)

        # self.vf.loadEnvironmentModel(Environement)

        self.ps.createLockedJoint('locked_finger_1', f'pandas/{self.arm_id}_finger_joint1', [0.035])
        self.ps.createLockedJoint('locked_finger_2', f'pandas/{self.arm_id}_finger_joint2', [0.035])
        self.ps.setConstantRightHandSide('locked_finger_1', True)
        self.ps.setConstantRightHandSide('locked_finger_2', True)

        
        self.ps.addPathOptimizer('SimpleShortcut')
        self.ps.addPathOptimizer('RandomShortcut')
        self.ps.addPathOptimizer('SimpleTimeParameterization')

        ps.setParameter('SimpleTimeParameterization/order', 2)
        ps.setParameter('SimpleTimeParameterization/safety',0.5)
        ps.setParameter('SimpleTimeParameterization/maxAcceleration',0.5)
        

        self.cg = ConstraintGraph(self.robot,"manipulation") #Une fonction pour reset les graphes
        factory = ConstraintGraphFactory(self.cg)
        factory.generate()


    def askQinit(self, msg):
        q_goal = msg.position.tolist()
        self.hpp_joint_names = msg.name

        # if len(q_goal) != len(self.hpp_joint_names):
        #     self.get_logger().error(f"Joint names ({len(self.hpp_joint_names)}) and position ({len(q_goal)}) doesn't have the same lenght !")
        #     return

        self.q_goal = self.setGripperValue(q_goal)

        self.get_logger().info("Waiting for service /joint_trajectory_controller/query_state...")
        if not self.state_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service not available! Exiting...")
            return

        request = QueryTrajectoryState.Request()
        now = self.get_clock().now()
        request.time = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])

        self.setupHPP()

        future = self.state_service.call_async(request)

        future.add_done_callback(self.solve)

    def solve(self, msg):

        q_init = msg.result().position.tolist()
        q_names = msg.result().name
        
        q_init = self.computeFullQinit(q_init, q_names)

        self.ps.setInitialConfig (q_init)
        self.ps.addGoalConfig (self.q_goal)

        self.cg.initialize()

        self.get_logger().info("Computing trajectory...")
        try:
            self.ps.solve()
        except Exception as e:
            self.get_logger().error(f"Planning FAILED : {e}")

        waypoints, times = self.ps.getWaypoints(0)

        trajectory_msg = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.joint_names = q_names
        trajectory.points = []

        for i in range(len(times)):
            print(times[i])
            wp = waypoints[i]
            point = JointTrajectoryPoint()
            point.positions = self.extractRobotConfig(wp, q_names, grip=wp[7])
            v = self.ps.derivativeAtParam(self.ps.numberPaths()-1, 1, times[i])
            point.velocities = self.extractRobotConfig(v, q_names)
            point.time_from_start = Duration(sec=int(times[i]), nanosec=int((times[i] - int(times[i])) * 1e9))

            trajectory.points.append(point)

        trajectory_msg.trajectory = trajectory

        self.controller.wait_for_server()
        goal_handle = self.controller.send_goal_async(trajectory_msg).result()
        self.get_logger().info("Trajectory sended")

    # ==========================================================
    # Following function need to be modified to be more generalist

    def setGripperValue(self, q, val=0.035):
        q[7] = val
        q[8] = val
        return q

    def extractRobotConfig(self, q, desired_joint_names, grip=0.0): 
        # Le param grip permet de passer la valeur pour le joint gripper. 
        # En position on envoie la valeur orginale.
        # En vitesse 0, car le gripper est statique.

        q_ros2 = q[0:7]
        if not self.no_grip:
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
    
    def computeFullQinit(self, q, joint_names):
        q_init = self.robot.getCurrentConfig()

        q_init[0:7] = q[0:7]

        if not self.no_grip:
            self.gripper_value = q[7] # ON sauvegarde la valeur finger_joint

        # Tri selon les noms
        # for i in range(len(q)):
        #     find = False
        #     if 'finger' in joint_names[i]:
        #         self.gripper_value[joint_names[i]] = q[i]
        #         continue
        #     for j in range(len(self.q_goal)):
        #         if joint_names[i] == self.hpp_joint_names[j]:
        #             q_init[j] = q[i] # We apply the value of the q_init to the right joint according to the user order for q_init
        #             find = True
        #             break
        #     if not find:
        #         self.get_logger().error(f"No joint {joint_names[i]} found in the q_goal provided by the user.\nThe ros2_control interface has : {joint_names}.\nThis will lead to incoherent behavior")

        return self.setGripperValue(q_init)
    
    # ==============================================================

class Ground (object):
  rootJointType = 'anchor'
  packageName = 'hpp_environments'
  urdfName = 'construction_set/ground'
  urdfSuffix = ""
  srdfSuffix = ""

class Environement(object):
    packageName = 'hpp_environments'
    urdfName = 'drawer_desk/desk'
    urdfSuffix = ""
    srdfSuffix = ""


def main(args=None):
    rclpy.init(args=args)

    hpp = HPPSimple()
    rclpy.spin(hpp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hpp.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




# self.ps.addOptimizer()







# print (ps.solve ())
# Pour recup la traj avec les points :  wp, times = ps.getWaypoints(pathId=0) times = [t1,t2,..ti..]
# POur trouver les vitesses ps.derivativeAtParam(pathId, 1, ti)

## Uncomment this to connect to a viewer server and play solution paths
# 
# v = vf.createViewer()
# from hpp.gepetto import PathPlayer
# pp = PathPlayer (v)

# pp (0)
# pp (1)

