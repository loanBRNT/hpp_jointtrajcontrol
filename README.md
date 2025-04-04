# HPP ROS2 CONTROL
![Docker](https://img.shields.io/badge/Docker-Supported-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)


Author : Loan Bernat. l.bernat@sileane.com

### Compatibility :

| hpp_interface | hpp_control | Robot tested
|-------|-------|-----|
| 1.0.2 | 1.0.2 | fr3 |
| 1.0.1 | 1.0.1 | fr3 |
| 1.0.0 | 1.0.0 | fr3, fer* |

(*) fer must work, you just need to change the urdf to include the mounted camera, if using the ADREM ones.

## Description
This repository contains two ROS2 packages :
- **hpp_control** : The ros2 control package to interface the software [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc/) (HPP) easly with the ros2_controller [Joint Trajectory Controller](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).
- **hpp_interface** : The ros2 package with message, service and action type for hpp_control.

The repository also provides a **Dockerfile** which allow you to have a base image already tested to run those packages on a separated container from the rest of your code. See [Docker Integration](#docker-integration) for more details.

⚠️ ***WARNING*** : These packages are thinked to be used with panda arm *fer* (Franka Emika Research) or *fr3* (Franka Research 3) in the ADREAM environment. To use on differents robots or environment, you need to change loaded URDF and SRDF and modify the `extractRobotConfig` function in utils.py.

❗ ***INFORMATION*** : To control robot in real time, you need to have other ros2 node launched and a Joint Trajectory Controller activated.

## Usage

In the hpp_control package you have two nodes and two exemple scripts.

### Examples

- **move_one_box.py** : Exemple of script to create a HPP problem and solve it.

- **move_cartesian.py** : Example of script to create an handle in the space and generate a configuration of the robot with the end-effector positioned at the handle.

From these script (using interactive python `python3 -i **.py`) you can aslo acess functions from **utils.py**. Those functions allow you to rapidly test and debug your script without the need to use a separated node. (Your simulation or real robot launch need to running and using Joint Trajectory Controller from ros2_control)
- **getRobotState()** connect to the robot and return its configuration. 
- **sendingTraj(problesolver, pathID, derivative, ARM_ID)** send the path corresponding to the pathID to a ARM_ID robot with only position (derivative=0), position + speed (derivative=1), position + speed + acceleration (derivative=2) 

### ROS2 Nodes

Both nodes take a ros parameter : arm_id ; Which correspond to the type of panda you are using (fer or fr3).

**sending_hpp** : Sending an already calculated trajectory to the ros2 controllers.

You can interact with the node using the `/hpp_node/sendTrajectory` service based on the `HppSendTrajectory` service type.

To use this node, you need to have a corbaserver running with trajectory already solved. For example, you can use the node to send the trajectory computed from **move_one_box.py**

**solving_hpp** : Standalone node to plan a trajecotry with HPP. Can take a configuration or an end-effector pose in input.

You can interact using ros2 actions `ConfigSolve` or `PoseSolve` if you want to wait the end of the trajectory. Or you can simply publish the desired pose on the topic `/hpp_node/fast_plan_to_q` to automatically calcul and send the trajectory.

## Docker Integration

To use those packages in a container, you just need to mount folders hpp_control and hpp_interface direclty into the container, setting network:'host' (to simplify) and setting the same ROS_DOMAINE_ID that the rest of your project.

This is an exemple of use inside a docker-compose file : 
```
hpp:
    build: 
      context: .
      dockerfile: hpp_jointtrajcontrol/Dockerfile
    network_mode: "host"
    container_name: hpp
    command: /bin/bash
    tty: true
    stdin_open: true
    volumes:
      - ./hpp_jointtrajcontrol/hpp_control:/ros2_ws/src/hpp_control
      - ./hpp_jointtrajcontrol/hpp_interface:/ros2_ws/src/hpp_interface
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /dev:/dev
    environment:
      QT_X11_NO_MITSHM: 1
      DISPLAY: $DISPLAY
      ROS_DOMAIN_ID: 10
      RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
    cap_add:
      - SYS_NICE
    ulimits:
      rtprio: 99
      rttime: -1
      memlock: 8428281856
```

If you are not familiar with docker or docker compose. Check the [official documentation](https://docs.docker.com/compose/)

## Support
Please contact me at l.bernat@sileane.com

## Contributing
You can contribute to this project by opening a merge request.

## License
Appache-2.0
