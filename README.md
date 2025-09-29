<div align="center">
<h1>🥋 PROJECT NEO</h1>
<h4>Getting He1nz to dodge bullets like the chosen one.</h4>

<a href="https://github.com/DominicZahn/Project_Neo/blob/main/docu/ProjectNeo.pdf">
    <img src="https://img.shields.io/badge/Report-green" alt="Report Link">
</a>
<a href="https://github.com/DominicZahn/Project_Neo/blob/main/docu/projectNeo/ProjectNeo/ProjectNeo.pdf">
    <img src="https://img.shields.io/badge/Presentation-blue" alt="Presentation Link">
</a>
</div>

# Quick Start
To get started recursively clone the repository with
```git
git clone --recursive https://github.com/DominicZahn/ROS2-Gazebo-Docker.git
```
It is important that the subrepositories [ROS2-Gazebo-Docker](https://github.com/DominicZahn/ROS2-Gazebo-Docker/tree/574c054fbd8337f6363a4b97bfed7b0e586321eb#) and [ros2_heinz](https://github.com/K-d4wg/ros2_heinz/tree/6d28be5d449fc5cbbe89e5be2110f219c400cbe9) are included.
## Setup and Docker Control
The general interaction with the Docker container can be seen in the subrepository [ROS2-Gazebo-Docker](https://github.com/DominicZahn/ROS2-Gazebo-Docker/blob/574c054fbd8337f6363a4b97bfed7b0e586321eb/README.md).

## Launch `Smith`
Inside the running docker the dodge example can be executed by launching all nodes with a single launch file.
```bash
ros2 launch dodge_it stability.launch.py sim:=rviz_manual
```
This command launches the following nodes
- center_of_mass
- support_polygon
- robot_state_publisher
- rviz2

Now the RViz window should appear and show how the optimization is solved step by step.
The time each step takes is highly depended on the computational capabilities of your system.
