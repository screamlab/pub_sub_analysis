# Joint Trajectory Publisher and Loss Calculator Subscriber

This project demonstrates a ROS 2 package containing two C++ nodes that work together to simulate message tracking and loss calculation using a custom publishing/subscribing mechanism. The publisher node sends `trajectory_msgs/msg/JointTrajectoryPoint` messages with a positions vector where each element is set to the same counter value. The subscriber node listens to the messages, calculates the package loss based on gaps in the counter value, and logs the following every second:
- Overall data count
- Overall loss count
- Overall loss percentage
- Current loss (messages lost in the most recent one-second interval)
- Standard deviation of loss per second (stdev)

When SIGINT (Ctrl+C) is received, the subscriber logs a final summary.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Requirements](#requirements)
- [Building the Project](#building-the-project)
- [Running the Nodes](#running-the-nodes)
  - [Publisher Node](#publisher-node)
  - [Subscriber Node](#subscriber-node)
- [Parameters](#parameters)
- [Project Structure](#project-structure)
- [License](#license)
- [Author](#author)



## Overview

This project consists of two ROS 2 nodes:

1. **pub**  
   - Publishes `trajectory_msgs/msg/JointTrajectoryPoint` messages.
   - The positions vector in each message is filled with a counter value.
   - Supports configurable publish rate, topic name, and number of positions per message.

2. **sub**  
   - Subscribes to the publisher’s topic.
   - Extracts the counter from the first element of the positions vector.
   - Calculates package loss as follows:  
     If the difference between consecutive counter values is greater than 1, the difference minus 1 is counted as lost.
   - Logs overall data count, loss count, loss percentage, loss per second, and the standard deviation of loss per second every 1 second.
   - Displays a final loss summary upon SIGINT (Ctrl+C).



## Features

- **Configurable Publisher:**
  - **publish_rate**: Message publish rate in Hz.
  - **publish_topic**: Topic name to publish messages.
  - **num_positions**: Number of elements in the positions vector.

- **Loss Calculation in Subscriber:**
  - Tracks the number of messages received and determines lost packages based on counter differences.
  - Computes loss per second and the standard deviation of loss per second samples.
  - Logs these statistics every second and upon shutdown.



## Requirements

- ROS 2 (any distribution that supports `rclcpp`; tested on Jazzy/Humble)
- C++ compiler with C++11 support
- ROS 2 packages: `rclcpp`, `trajectory_msgs`



## Building the Project

1. **Create a ROS 2 workspace** (if you don’t already have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

1. **Clone or copy this project** into your workspace (for example, within a folder named `joint_trajectory_project`).

2. **Build the workspace** with colcon:

   ```
   cd ~/ros2_ws
   colcon build --packages-select <your_package_name>
   ```

3. **Source the workspace**:

   ```
   source install/setup.bash
   ```



## Running the Nodes

### Publisher Node

To run the publisher node with default parameters:

```
ros2 run pub_sub_analysis pub
```

You can override the default parameters with the `--ros-args` flag. For example, to set the publish rate to 10 Hz, the topic name to `/right`, and the number of positions to 11:

```
ros2 run pub_sub_analysis pub --ros-args -p publish_rate:=10.0 -p publish_topic:=/right -p num_positions:=11
```

### Subscriber Node

To run the subscriber node with default settings:

```
ros2 run pub_sub_analysis sub
```

To override the subscription topic (if you ran the publisher with a custom topic):

```
ros2 run pub_sub_analysis sub --ros-args -p subscribe_topic:=/right_republish
```

The subscriber will log loss statistics every 1 second, and upon SIGINT (Ctrl+C) it will print a final summary.



## Parameters

### Publisher Node Parameters

- **publish_rate** (double):
   The rate at which messages are published (in Hz).
   *Default:* `100.0`
- **publish_topic** (string):
   The topic where the messages are published.
   *Default:* `"/right_arm"`
- **num_positions** (int):
   The number of elements in the positions vector in each message.
   *Default:* `6`

### Subscriber Node Parameters

- **subscribe_topic** (string):
   The topic to subscribe to for incoming messages.
   *Default:* `"/right_arm_republish"`



## Project Structure

```
├── CMakeLists.txt
├── LICENSE
├── package.xml
├── README.md
├── scripts
│   ├── pre-commit.hook
│   └── trigger.sh
└── src
    ├── pub.cpp       # Publisher node source code.
    └── sub.cpp       # Subscriber node source code.
```

## License

This project is licensed under the MIT license.



## Author

This sample project was created as a tutorial demonstration for developing simple ROS 2 publisher/subscriber nodes with message loss detection and statistics.



---

This `README.md` file gives users a clear explanation of the project, how to build and run the nodes, the configurable parameters, and the overall project structure.