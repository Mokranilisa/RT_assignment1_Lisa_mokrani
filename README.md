# ROS Turtle Simulation Project

This project demonstrates basic control and monitoring of multiple turtles in the ROS (Robot Operating System) environment using the `turtlesim` package. The system includes two primary functionalities:
- **Spawning and controlling turtles** via velocity commands.
- **Monitoring the distance** between two turtles and ensuring they stay within a predefined boundary.

## Installation

Follow these steps to install and set up the project:

1. **Install ROS**: Follow the official [ROS installation guide](https://www.ros.org/install/).
2. **Clone the repository**:
    ```bash
    git clone https://github.com/Mokranilisa/RT_assignment1_Lisa_mokrani
    ```
3. **Build the workspace**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
4. **Source the workspace**:
    ```bash
    source devel/setup.bash
    ```

## Running the Simulation

### Node 1: Turtle Spawning and Control
#### Python Implementations
1. Start the ROS core:
    ```bash
    roscore
    ```
2. Run the script to spawn and control the turtles:
    ```bash
    rosrun <package_name>  node1.py
    ```
3. You will see a menu with options:
    - **1**: Spawn `turtle2`.
    - **2**: Send velocity commands to `turtle1` or `turtle2`.
    - **3**: Exit the program.
#### C++ Implementations
1.Compile the workspace with:

```bash 
catkin_make
 ```

2.Run node1 (C++ implementation):

```bash 
rosrun <package_name> node1
 ```
### Node 2: Distance Monitoring and Boundary Checking
#### Python Implementations
1. Run the second script to monitor the distance and boundaries:
    ```bash
    rosrun <package_name>  node2.py
    ```
2. This node will continuously check the distance between `turtle1` and `turtle2`. If the turtles get too close or approach the boundary, they will stop automatically.

## Features

- **Turtle Spawning**: Dynamically spawn a second turtle in the simulation.
- **Velocity Control**: Send linear and angular velocity commands to the turtles.
- **Distance Calculation**: Calculate and publish the distance between the two turtles.
- **Boundary Checking**: Stop turtles if they approach predefined boundaries.
- 
#### C++ Implementations
1.Compile the workspace with:

 ```bash 
 catkin_make
```

2.Run node1 (C++ implementation):

```bash 
rosrun <package_name> node2
 ```
## Pseudo Code

### Node 1: Spawning and Controlling Turtles(Python & C++)

```bash
Start
  Initialize ROS node
  While True:
    Display options to user:
      1. Spawn turtle2
      2. Send velocity command
      3. Exit
    Get user choice
    If choice is 1:
      Spawn turtle2 at position (5, 5)
    Else If choice is 2:
      Prompt for turtle name and velocity inputs
      Send velocity command for 1 second
    Else If choice is 3:
      Exit program
End
 ```

### Node 2: Distance Monitoring and Boundary Checking (Python & C++)

```bash
Start
  Initialize ROS node
  Subscribe to pose data of turtle1 and turtle2
  Wait for turtle2 to be spawned
  While True:
    Calculate distance between turtle1 and turtle2
    If distance < threshold:
      Stop both turtles
    If turtles approach boundaries:
      Stop the respective turtle
    Publish the distance
End
```

## Troubleshooting

If you face issues with spawning or controlling turtles, ensure the following:
- Ensure the ROS core (`roscore`) is running before starting the scripts.
- Double-check the velocity inputs for correct formatting.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
