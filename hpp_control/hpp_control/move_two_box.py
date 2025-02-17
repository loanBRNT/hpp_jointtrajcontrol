# Import libraries and load robots. {{{1

# Import.
from math import sqrt

from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    ProblemSolver,
    Rule,
)
from hpp.corbaserver.manipulation.pr2 import Robot
from hpp.gepetto import PathPlayer  # noqa: F401
from hpp.gepetto.manipulation import ViewerFactory


BOX_RANGE = [-2.65, -4.4, -2.15, -3.75] #TOP LEFT BOTTOM RIGHT

ARM_ID = 'fer'


loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()



# Specify path for robot urdf and srdf files
Robot.urdfFilename = f"/ros2_ws/src/hpp_control/robots/{ARM_ID}.urdf"

Robot.srdfFilename = f"/ros2_ws/src/hpp_control/robots/{ARM_ID}.srdf"

class Box:
    rootJointType = "freeflyer"
    packageName = "hpp_tutorial"
    urdfName = "box"
    urdfSuffix = ""
    srdfSuffix = ""


class Environment:
    packageName = "hpp_tutorial"
    urdfName = "kitchen_area"
    urdfSuffix = ""
    srdfSuffix = ""


robot = Robot("robot", "panda", rootJointType="anchor")
robot.setRootJointPosition("panda", [-2.379, -4.19, 0.733, 0,0,0,1])
ps = ProblemSolver(robot)
# ViewerFactory is a class that generates Viewer on the go. It means you can
# restart the server and regenerate a new windows.
# To generate a window:
# vf.createViewer ()
vf = ViewerFactory(ps)

vf.loadObjectModel(Box, "box1")
vf.loadObjectModel(Box, "box2")
vf.loadEnvironmentModel(Environment, "kitchen_area")

robot.setJointBounds("box1/root_joint", [-5.5, -1.5, -5.5, -2.5, 0, 1.5])
robot.setJointBounds("box2/root_joint", [-5.5, -1.5, -5.5, -2.5, 0, 1.5])

# Add gripper
robot.client.manipulation.robot.addGripper\
    (f"panda/{ARM_ID}_hand_tcp", 'panda/gripper', [0,0,0,sqrt(2)/2,0,sqrt(2)/2,0], 0.1)

# Initialization.
ps.selectPathPlanner('StatesPathFinder')
ps.addPathOptimizer('SimpleShortcut')
if ps.loadPlugin('manipulation-spline-gradient-based.so') :
    ps.addPathOptimizer('SplineGradientBased_bezier1')
    print("Spline bezier 1 added to optimizer")

ps.createLockedJoint('locked_finger_1', f"panda/{ARM_ID}_finger_joint1", [0.035])
ps.createLockedJoint('locked_finger_2', f"panda/{ARM_ID}_finger_joint2", [0.035])
ps.setConstantRightHandSide('locked_finger_1', True)
ps.setConstantRightHandSide('locked_finger_2', True)

# Set parameters.
# robot.client.basic.problem.resetRoadmap ()
ps.setErrorThreshold(1e-3)
ps.setMaxIterProjection(40)

# Generate initial and goal configuration.
q_init = robot.getCurrentConfig()
q_init[0:9] = [ -1.5413388441356696e-07, -0.7854117616646089, 1.3035790034226983e-12,
               -2.3562320791148843, -5.016925061582818e-11, 1.5708301393693362, 0.7853981633974482, 0.035, 0.035]
q_goal = q_init[::]

rank = robot.rankInConfiguration["box1/root_joint"]
q_init[rank : rank + 3] = [-2.5, -3.75, 0.746]
q_goal[rank : rank + 3] = [-2.5, -4.5, 0.746]

# Put box in right orientation
q_init[rank + 3 : rank + 7] = [0, -sqrt(2) / 2, 0, sqrt(2) / 2]
q_goal[rank + 3 : rank + 7] = q_init[rank + 3 : rank + 7] 

rank = robot.rankInConfiguration["box2/root_joint"]
q_init[rank : rank + 3] = [-2.15, -3.75, 0.746]
q_goal[rank : rank + 3] = [-2.15, -4.5, 0.746]

# Put box in right orientation
q_init[rank + 3 : rank + 7] = [0, -sqrt(2) / 2, 0, sqrt(2) / 2]
q_goal[rank + 3 : rank + 7] = q_init[rank + 3 : rank + 7] 


# Create the constraints.




# Create the constraint graph.
# Define the set of grippers used for manipulation
grippers = [
    "panda/gripper",
]
# Define the set of objects that can be manipulated
objects = [
    "box1",
    "box2",
]
# Define the set of handles for each object
handlesPerObject = [
    [
        "box1/handle2",

    ],
    [
        "box2/handle2",
    ],
]
# Define the set of contact surfaces used for each object
contactSurfacesPerObject = [
    [
        "box1/box_surface",
    ],
    [
        "box2/box_surface",
    ],
]
# Define the set of contact surfaces of the environment used to put objects
envContactSurfaces = [
    "kitchen_area/pancake_table_table_top",
]
# Define rules for associating grippers and handles (here all associations are
# allowed)
rules = [
    Rule([".*"], [".*"], True),
]

cg = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory(cg)
factory.setGrippers(grippers)
factory.environmentContacts(envContactSurfaces)
factory.setObjects(objects, handlesPerObject, contactSurfacesPerObject)
factory.setRules(rules)
factory.generate()
#cg.addConstraints(graph=True, constraints=Constraints(numConstraints=locklhand))
cg.initialize()

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

# uncomment to solve
# ps.solve()

# Path optimization uncomment to optimize
#
# ps.loadPlugin('manipulation-spline-gradient-based.so')
# ps.addPathOptimizer('SplineGradientBased_bezier1')
# ps.optimizePath(0)

# display in gepetto-gui
# v = vf.createViewer ()
