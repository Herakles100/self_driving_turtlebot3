# ASelf Driving Turtlbot3 Project

## About Us

We are Team-2 of AuE893 Autonomy Science and Systems (Spring 2021). Our team consists of Nitin Madhok, Prakhar Gupta, Ryan Nguyen, Shengli Xu, and Sreekar Malladi.

## Experiment Results

- Simulation in Gazebo
  ![Visualisation](./gifs/final_project_gazebo.gif)

- Result in Real World
  ![Visualisation](./gifs/final_project_real-world.gif)

## Getting started

### Prerequisites

- Ubuntu 20.04
- Ros Noetic
- ROS1
- TurtleBot3 packages
- TurtleBot3 Burger
- Python3
- OpenCV
- Apriltag
- git
- TensorFlow 2.x

### Clone the project

```bash
$ cd ~/catkin_ws/src
$ git clone git@github.com:AuE893-Spring21-Team-2/aue_finals.git
```

### Build code in your catkin workspace

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

### Make the node executable

```bash
$ cd ~/catkin_ws/src/aue_finals/src/scripts
$ chmod u+x *.py
```

### Part I: Run Project in Gazebo

```bash
$ roslaunch aue_finals turtlebot3_autonomy_final_simulation.launch
```

### Part II: Run Project in Real-World

- Run roscore on Remote PC

  ```bash
  $ roscore
  ```

- Bring up TurtleBot3 Burger

  ```bash
  $ ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
  $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```

- Run project code
  ```bash
  $ roslaunch aue_finals turtlebot3_autonomy_final_real.launch
  ```
