# Import libraries and load robots.

# Import.
from math import sqrt
import os

from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import (
    Client,
    Constraints,
    ConstraintGraph,
    ConstraintGraphFactory,
    ProblemSolver,
    Rule,
)
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.gepetto import PathPlayer  # noqa: F401
from hpp.gepetto.manipulation import ViewerFactory
from ament_index_python.packages import get_package_share_directory

# Import to interact with ROS2 controller. 
# If you want to make a lot of call to these function, please considerer using sending_hpp node instead.
from utils import getRobotState, sendingTraj



ARM_ID = 'fr3'


loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()
pkg_path = get_package_share_directory("hpp_control")

# Specify path for robot urdf and srdf files
Robot.urdfFilename = os.path.join(pkg_path, "urdf" , f"{ARM_ID}.urdf")
Robot.srdfFilename = os.path.join(pkg_path, "srdf" , f"{ARM_ID}.srdf")

robot = Robot("robot", "pandas", rootJointType="anchor")

ps = ProblemSolver(robot)

vf = ViewerFactory(ps)

# Add gripper
robot.client.manipulation.robot.addGripper\
    (f"pandas/{ARM_ID}_hand_tcp", 'pandas/gripper', [0,0,0,sqrt(2)/2,0,sqrt(2)/2,0], 0.1)

# Initialization.
#ps.selectPathPlanner('StatesPathFinder')
ps.addPathOptimizer('SimpleShortcut')
ps.addPathOptimizer('RandomShortcut')
ps.addPathOptimizer('SimpleTimeParameterization')

ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/safety',0.3)
ps.setParameter('SimpleTimeParameterization/maxAcceleration',0.2)

# if ps.loadPlugin('manipulation-spline-gradient-based.so') :
#     ps.addPathOptimizer('SplineGradientBased_bezier1')
#     print("Spline bezier 1 added to optimizer")

ps.createLockedJoint('locked_finger_1', f"pandas/{ARM_ID}_finger_joint1", [0.035])
ps.createLockedJoint('locked_finger_2', f"pandas/{ARM_ID}_finger_joint2", [0.035])
ps.setConstantRightHandSide('locked_finger_1', True)
ps.setConstantRightHandSide('locked_finger_2', True)

# Set parameters.
# robot.client.basic.problem.resetRoadmap ()
# ps.setErrorThreshold(1e-3)
# ps.setMaxIterProjection(40)

# Generate initial and goal configuration.
q_init = robot.getCurrentConfig()
q_init[0:9] = [ 0.0011392894365677708, -0.785233599521887, 0.0006221224673022915, -2.373483112932502, 
                0.003281429835572088, 1.559707000546985, 0.7660253966665929, 0.035, 0.035]
q_goal = q_init[::]
q_goal[0] = 1.5



# Create the constraints.




# Create the constraint graph.
# Define the set of grippers used for manipulation
grippers = [
    "pandas/gripper",
]
# Define the set of objects that can be manipulated
objects = [
    'box1'
]
# Define the set of handles for each object
handlesPerObject = [
    ["box1/handle2"]
]
# Define the set of contact surfaces used for each object
contactSurfacesPerObject = [
    ['box1/box_surface'],
]

envContactSurfaces = [
    "pandas/top",
]

# Define rules for associating grippers and handles (here all associations are
# allowed)
rules = [
    Rule([".*"], [".*"], True),
]

cg = ConstraintGraph(robot, "manipulation")
factory = ConstraintGraphFactory(cg)
# factory.setGrippers(grippers)
# factory.environmentContacts(envContactSurfaces)
# factory.setObjects(objects, handlesPerObject, contactSurfacesPerObject)
# factory.setRules(rules)
factory.generate()
#cg.addConstraints(graph=True, constraints=Constraints(numConstraints=locklhand))
# ps.createTransformationConstraint("move-vertical", "", "box1/root_joint", [0,0,0,0,0,0,1], [True, True, False, False, False, True])
# ps.setConstantRightHandSide("move-vertical", False)
# cg.addConstraints(edge="pandas/gripper > box1/handle2 | f_23", constraints = Constraints(numConstraints=["move-vertical"]))
# cg.addConstraints(edge="pandas/gripper < box1/handle2 | 0-0_32", constraints = Constraints(numConstraints=["move-vertical"]))

cg.initialize()

# pose = [0.13984079658985138, 0.0860714316368103, 0.05416488468647003, 0, sqrt(2)/2, 0, sqrt(2)/2]

pose = [0.251874303817749, 0.2739804828166962, 0.06485912144184112, 
-0.34917425900677534, -0.8949468121176025, 0.23254299630382577, 0.1518923803146453]


robot.client.manipulation.robot.addHandle('pandas/support_link','moveTo',pose, 0.1, 6 * [True])
print(pose)

#robot.client.manipulation.robot.setHandlePositionInJoint('moveTo',pose)
cg.createPreGrasp('preGrasp', 'pandas/gripper','moveTo') # grasp et projeter comme avec mplib
cg.createGrasp('grasp','pandas/gripper','moveTo')

p = ps.client.basic.problem.getProblem()

r = p.robot()

solverGrasp = ps.client.basic.problem.createConfigProjector(r,'graspSolver', 1e-6, 40)
solverPreGrasp = ps.client.basic.problem.createConfigProjector(r,'preGraspSolver', 1e-6, 40)

constraintGrasp = ps.client.basic.problem.getConstraint('grasp')
constraintPreGrasp = ps.client.basic.problem.getConstraint('preGrasp')

solverGrasp.add(constraintGrasp, 1)
solverPreGrasp.add(constraintPreGrasp, 1)

for i in range(1000):
    q = robot.shootRandomConfig()
    if i<=50:
        q = q_init
    res, q1 = solverPreGrasp.apply(q)
    if res:
        find = False
        for j in range(100):
            res, q2 = solverGrasp.apply(q1)
            if res:
                find = True
                break
        if find:
            break
print(f"{res} in {i}")

#constraint.deleteThis()

p.deleteThis()
solverPreGrasp.deleteThis()
solverGrasp.deleteThis()

ps.setInitialConfig(q_init)
ps.addGoalConfig(q1)

ps.solve()

ps.setInitialConfig(q1)
ps.resetGoalConfigs()
ps.addGoalConfig(q2)

ps.solve()
ps.concatenatePath(3, 7)