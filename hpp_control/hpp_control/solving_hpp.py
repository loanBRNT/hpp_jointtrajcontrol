# Author : Loan BERNAT (l.bernat@sileane.com)

from hpp.corbaserver.manipulation import Robot
from hpp.corbaserver import loadServerPlugin, shrinkJointRange
from hpp.corbaserver.manipulation import Robot, \
    newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Constraints
from hpp.gepetto.manipulation import ViewerFactory
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from control_msgs.srv import QueryTrajectoryState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
from hpp_interface.action import PoseSolve, ConfigSolve, InvTrajSolve, TestGrasp

from ament_index_python.packages import get_package_share_directory
import os
from math import sqrt
from .utils import generateMessage

# Example of commande
# ros2 topic pub /hpp_node/fast_plan_to_q sensor_msgs/msg/JointState "{position: [0.0, 0.5, 0.4, -1.0, 0.0, 1.0, 0.0, 1.0, 1.0]}" --once

def pose_to_list(p):
    return [
        p.position.x,
        p.position.y,
        p.position.z,
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w,
    ]
    
def list_to_pose(lst):
    p = Pose()
    p.position.x = lst[0]
    p.position.y = lst[1]
    p.position.z = lst[2]
    p.orientation.x = lst[3]
    p.orientation.y = lst[4]
    p.orientation.z = lst[5]
    p.orientation.w = lst[6]
    return p

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

        self.declare_parameter('arm_id', '')
        self.arm_id = self.get_parameter('arm_id').value

        self.declare_parameter('load_grip', False)
        self.no_grip = not self.get_parameter('load_grip').value

        if self.arm_id not in self.SUPPORTED_ROBOTS:
            raise RuntimeError(f"ERROR : Bad argument.\nRobot {self.arm_id} is not supported. Avalaible robots {self.SUPPORTED_ROBOTS}")

        self.joint_names = [f"{self.arm_id}_joint{i}" for i in range(1,8)]

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
        self.cb_group = ReentrantCallbackGroup()

        self.action_ee_pose = ActionServer(
            self, 
            PoseSolve, 
            "/hpp_node/plan_trajectory_to_ee_pose",
            execute_callback=self.plan_ee_pose,
            goal_callback=self.goal_callback,
            callback_group=self.cb_group
        )

        self.action_grasp = ActionServer(
            self, 
            TestGrasp, 
            "/hpp_node/check_grasps",
            execute_callback=self.checkGrasps,
            goal_callback=self.goal_callback,
            callback_group=self.cb_group
        )

        self.action_q_pose = ActionServer(
            self, 
            ConfigSolve, 
            '/hpp_node/plan_trajectory_to_q_config',
            execute_callback=self.plan_q_config,
            goal_callback=self.goal_callback,
            callback_group=self.cb_group
        )

        self.action_inv_last_traj = ActionServer(
            self, 
            InvTrajSolve, 
            "/hpp_node/inv_trajectory",
            execute_callback=self.inv_last_traj,
            goal_callback=self.goal_callback,
            callback_group=self.cb_group
        )
        
        self.action_already_running = False

        self.subscriber = self.create_subscription(JointState, '/hpp_node/fast_plan_to_q', self.askQinit, 2)

        # Receive the current config of the robot
        self.state_service = self.create_client(QueryTrajectoryState, '/joint_trajectory_controller/query_state')

    def goal_callback(self, goal_request):
        """Accepts or rejects an incoming goal"""
        if self.action_already_running:
            self.get_logger().info("New request received, but a node is already running.")
            return GoalResponse.REJECT
        self.action_already_running = True
        self.get_logger().info("GOAL ACCEPTED")
        return GoalResponse.ACCEPT  # Always accept for now

    def verifyConfig(self, q, eps=0.98):
        # joint_names = self.robot.getJointNames()
        # for i in range(len(q)):
        #     if "finger" in joint_names[i]:
        #         continue 
        #     l = self.robot.getJointBounds(joint_names[i])
        #     limit = ( l[0] + abs(l[0])*(1-eps) , l[1] - abs(l[1])*(1-eps))
        #     if q[i] < limit[0] or q[i] > limit[1]:
        #         self.get_logger().warn(f"Refused pose because joint {joint_names[i]} with {q[i]} not in limit : {limit} for {l}")
        #         return False
        return True


    def setupHPP(self):
        newProblem()

        self.traj_in_memory = False

        self.robot = Robot("robot", "pandas", rootJointType="anchor")
        self.robot.setRootJointPosition("pandas",[0,0,0.4,0,0,0,1]) # A changer selon l'env
        shrinkJointRange(self.robot, [f'pandas/{self.arm_id}_joint{i}' for i in range(1,8)],0.95)
        self.ps = ProblemSolver(self.robot)
        # self.ps.loadPlugin("spline-gradient-based.so")
        self.ps.setMaxIterPathPlanning(2000)

        self.robot.client.manipulation.robot.addGripper\
            (f"pandas/{self.arm_id}_hand_tcp", 'pandas/gripper', [0,0,0,sqrt(2)/2,0,sqrt(2)/2,0], 0.05)
        
        grippers = [
            "pandas/gripper",
        ]

        self.ps.createLockedJoint('locked_finger_1', f'pandas/{self.arm_id}_finger_joint1', [0.035])
        self.ps.createLockedJoint('locked_finger_2', f'pandas/{self.arm_id}_finger_joint2', [0.035])
        self.ps.setConstantRightHandSide('locked_finger_1', False)
        self.ps.setConstantRightHandSide('locked_finger_2', False)

        
        self.ps.addPathOptimizer('SimpleShortcut')
        self.ps.addPathOptimizer('RandomShortcut')
        # self.ps.addPathOptimizer("SplineGradientBased_bezier3")
        self.ps.addPathOptimizer('SimpleTimeParameterization')
        # If you add another optimizer change the path id in the solve() --> getWaypoints().

        self.ps.setParameter('SimpleTimeParameterization/order', 2)
        self.ps.setParameter('SimpleTimeParameterization/safety',0.8)
        self.ps.setParameter('SimpleTimeParameterization/maxAcceleration',0.8)
        

        self.cg = ConstraintGraph(self.robot,"manipulation") #Une fonction pour reset les graphes
        factory = ConstraintGraphFactory(self.cg)
        factory.setGrippers(grippers)
        factory.generate()
        self.cg.addConstraints(constraints=Constraints(numConstraints=["locked_finger_1","locked_finger_2"]))
        self.cg.initialize()


    def askQinit(self, msg):
        q_goal = msg.position.tolist()
        self.hpp_joint_names = msg.name


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

        future.add_done_callback(self.getQfromMsg)

    async def inv_last_traj(self, goal_handle):

        response = InvTrajSolve.Result()
        if self.traj_in_memory:
            response.success = await self.sendTrajectory(inv=True)
        else :
            response.success = False
            self.get_logger().error(f"No trajectory in memory. Cannot invert nothing")

        goal_handle.succeed()
        self.action_already_running = False

        return response

    async def plan_ee_pose(self, goal_handle):
        self.setupHPP()
        self.pose_goal = goal_handle.request.pose
        self.degrees = goal_handle.request.flags
        if len(goal_handle.request.flags) != 6:
            self.degrees = 6 * [True]
        q_init = goal_handle.request.q_init.position.tolist()
        self.q_init = self.setGripperValue(q_init)
        
        pose = [
                self.pose_goal.position.x,
                self.pose_goal.position.y,
                self.pose_goal.position.z,
                self.pose_goal.orientation.x,
                self.pose_goal.orientation.y,
                self.pose_goal.orientation.z,
                self.pose_goal.orientation.w
                ]

        self.get_logger().info(f"pose: {pose}")
        self.get_logger().info(f"q_init : {q_init}")

        res, self.q_goal, q2 = self.computeConfigFromPose(q_init, pose, freedom=self.degrees, pre_grasp=goal_handle.request.should_pre_grasp)

        response = PoseSolve.Result()
        response.success = False

        if res:
            if not goal_handle.request.should_pre_grasp:      
                response.success = await self.solve()
            else:
                res = await self.solve(send=False)
                if res:
                    self.q_init = self.q_goal
                    self.q_goal = q2
                    res = await self.solve(send=False)
                    if res:
                        self.ps.concatenatePath(3,7)
                        response.success = await self.sendTrajectory()
        else:
            self.get_logger().error("No config founded")
            self.action_already_running = False
            goal_handle.abort()
            return response


        self.action_already_running = False
        goal_handle.succeed()

        return response

    def jointstate_to_ordered_positions(
        self,
        joint_state
    ):
        name_to_pos = dict(zip(joint_state.name, joint_state.position))

        ordered_positions = []
        for name in self.joint_names:
            if name not in name_to_pos:
                raise KeyError(f"Joint '{name}' not found in JointState")
            ordered_positions.append(name_to_pos[name])

        return ordered_positions

    async def plan_q_config(self, goal_handle):
        self.setupHPP()
        q_init = self.jointstate_to_ordered_positions(goal_handle.request.q_init)
        q_goal = self.jointstate_to_ordered_positions(goal_handle.request.q_goal)

        self.q_goal = self.setGripperValue(q_goal)
        self.q_init = self.setGripperValue(q_init)

        response = ConfigSolve.Result()
        
        response.success = await self.solve()

        self.action_already_running = False
        goal_handle.succeed()

        return response

    async def checkGrasps(self, goal_handle):
        self.setupHPP()
        valid_grasps = self.checkValidGrasp(goal_handle.request.grasps, goal_handle.request.q_init.position.tolist())

        response = TestGrasp.Result()
        response.valid_grasps = [list_to_pose(p) for p in valid_grasps]
        self.action_already_running = False
        goal_handle.succeed()
        return response


    def checkValidGrasp(self, grasp_list, q_init, nb_try=1000):
        self.robot.client.manipulation.robot.addHandle('pandas/support_link','moveTo',[0,0,0,0,0,0,1], 0.1, 6*[True])
        
        p = self.ps.client.basic.problem.getProblem()
        r = p.robot()
        
        seuil = nb_try // 10
        out = []
        for pose_msg in grasp_list:
            q = q_init.copy()
            pose = pose_to_list(pose_msg)
            self.get_logger().info(str(pose))
            self.robot.client.manipulation.robot.setHandlePositionInJoint("moveTo", pose)
            self.cg.createGrasp('grasp','pandas/gripper','moveTo')
            solverGrasp = self.ps.client.basic.problem.createConfigProjector(r,'graspSolver', 1e-6, 40)
            constraintGrasp = self.ps.client.basic.problem.getConstraint('grasp')
            solverGrasp.add(constraintGrasp, 1)
            for i in range(nb_try):
                if i >= seuil:
                    q = self.robot.shootRandomConfig()
                res, q1 = solverGrasp.apply(q)
                
                if res:
                    if self.verifyConfig(q1):
                        self.get_logger().info("Grasp valid")
                        out.append(pose)
                        break
            else:
                self.get_logger().warn("Grasp Invalid")

            if len(out) > 4:
                break
        
        p.deleteThis()
        solverGrasp.deleteThis()

        return out

    async def getQfromMsg(self, msg):
        q_init = msg.result().position.tolist()
        q_names = msg.result().name
        
        self.q_init = self.computeFullQinit(q_init, q_names)

        ssuccess = await self.solve()

    async def sendTrajectory(self, inv=False):
        self.get_logger().info("Getting informations from trajectory...")
        try:
            waypoints, times = self.ps.getWaypoints(3)
        except Exception as e:
            self.get_logger().error(f"ERROR : {e}")
            return False
        
        trajectory_msg = FollowJointTrajectory.Goal()

        trajectory_msg.trajectory = generateMessage(self.ps,waypoints,times,2,3,self.arm_id, inv=inv)

        self.controller.wait_for_server()
        goal_future = self.controller.send_goal_async(trajectory_msg)
        self.get_logger().info("Trajectory sent, waiting for result...")

        goal_handle = await goal_future

        result_future = goal_handle.get_result_async()
        result = await result_future

        self.get_logger().info("Trajectory Succeeded")

        result = result.result

        self.get_logger().info(f"Execution ended with value : {result.error_code}")
        return result.error_code == 0

    async def solve(self,send=True):
        if self.q_goal == None or self.q_init == None:
            self.get_logger().error("ERROR : Q_GOAL or Q_INIT is none")

        self.q_init = self.setGripperValue(self.q_init)
        self.q_goal = self.setGripperValue(self.q_goal)

        self.ps.resetGoalConfigs()
        self.ps.setInitialConfig(self.q_init)
        self.ps.addGoalConfig(self.q_goal)
        self.cg.initialize()

        self.get_logger().info(f"Computing trajectory {self.q_init} to {self.q_goal}...")

        try:
            self.traj_in_memory = True
            self.ps.solve()
        except Exception as e:
            self.get_logger().error(f"ERROR : {e}")
            return False

        if send:
            return await self.sendTrajectory()
        
        return True
    

    '''
    Return a config with the end effector at the pose.
    '''
    def computeConfigFromPose(self, q, pose, pre_grasp=False, nb_try=1000, freedom=6*[True]):
        self.robot.client.manipulation.robot.addHandle('pandas/support_link','moveTo',pose, 0.1, freedom)

        self.cg.createGrasp('grasp','pandas/gripper','moveTo')

        p = self.ps.client.basic.problem.getProblem()
        r = p.robot()

        solverGrasp = self.ps.client.basic.problem.createConfigProjector(r,'graspSolver', 1e-6, 40)
        constraintGrasp = self.ps.client.basic.problem.getConstraint('grasp')
        solverGrasp.add(constraintGrasp, 1)

        q_init = q
        seuil = nb_try // 5

        q1 = []
        q2 = []

        if pre_grasp:
            self.cg.createPreGrasp('preGrasp','pandas/gripper','moveTo')
            solverPreGrasp = self.ps.client.basic.problem.createConfigProjector(r,'preGraspSolver', 1e-6, 40)
            constraintPreGrasp = self.ps.client.basic.problem.getConstraint('preGrasp')
            solverPreGrasp.add(constraintPreGrasp, 1)

            for i in range(nb_try): 
                if i>=seuil:
                    q_init = self.robot.shootRandomConfig()
                res, q1 = solverPreGrasp.apply(q_init)
                if res:
                    if self.verifyConfig(q1):
                        find = False
                        for j in range(100):
                            res, q2 = solverGrasp.apply(q1)
                            if res:
                                if self.verifyConfig(q2):
                                    find = True
                                    break
                        if find:
                            break
                else:
                    self.get_logger().info(f"computeConfigFromPose : fail to project at it {i}")

            solverPreGrasp.deleteThis()

        else:
            for i in range(nb_try):
                if i >= seuil:
                    q_init = self.robot.shootRandomConfig()
                res, q1 = solverGrasp.apply(q_init)
                if res:
                    if self.verifyConfig(q1):
                        break
        
        p.deleteThis()
        solverGrasp.deleteThis()

        return res, q1, q2


    # ==========================================================
    # Following function need to be modified to be more generalist

    def setGripperValue(self, q, val=0.035):
        if len(q) < 8:
            q.append(val)
            q.append(val)
        else:
            q[7] = val
            q[8] = val
        return q
    
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

    node = HPPSimple()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


