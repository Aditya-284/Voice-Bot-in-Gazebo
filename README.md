# 🤖 Voice-Controlled Robot in ROS 2 Humble with Gazebo

This project demonstrates how to **control a differential drive robot using voice commands** in a **simulated Gazebo environment** using **ROS 2 Humble**.

---

## 🧠 Project Overview

The system includes:

- A **custom robot** defined in URDF/XACRO format.
- A **Gazebo simulation** with a custom world.
- A **voice control node** that interprets spoken commands and sends movement commands (`/cmd_vel`) to the robot.

---

## 🗂️ Project Structure

robot_sim/

├── urdf/

│ └── robot_model.urdf.xacro

├── world/

│ └── custom_world.world

├── launch/

│ └── launch_robot.xml

├── scripts/

│ └── voice_node.py

├── CMakeLists.txt

└── package.xml


## 🚀 Getting Started

### ✅ Prerequisites

Make sure you have the following installed:

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- `gazebo_ros` package:
  ```bash
  sudo apt install ros-humble-gazebo-ros-pkgs
Python libraries:
 ```bash
 pip install SpeechRecognition PyAudio
 ```

### 🛠️ Build the Workspace
1) Clone the repository into your ROS 2 workspace:
  ```bash
  cd ~/ros2_project/src
  git clone https://github.com/yourusername/voice_robot_sim.git
  cd ..
  ```

2) Build the workspace:
 ```bash
 colcon build
 ```

3) Source the setup file:
 ```bash
 source install/setup.bash
 ```

### 🧪 Run the Simulation
🟢 Step 1: Launch Robot + Gazebo + Voice Node
```bash
ros2 launch robot_sim launch_robot.xml
```
Make sure your microphone is enabled and working.

### 🗣️ Voice Commands Supported

| Command    | Action        |
| ---------- | ------------- |
| `forward`  | Move forward  |
| `backward` | Move backward |
| `left`     | Turn left     |
| `right`    | Turn right    |
| `stop`     | Stop movement |


### 🧾 Important Notes
1) Your robot_model.urdf.xacro must be syntactically correct and complete.

2) launch_robot.xml must properly include Gazebo, spawn the robot, and run the voice node.

3) Run the launch file after building and sourcing the workspace.

4) Test your microphone using a record or any other tool before launching.

### 📚 References
[ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)

[Gazebo ROS Packages](http://gazebosim.org/tutorials?tut=ros_overview)

[SpeechRecognition Docs](https://pypi.org/project/SpeechRecognition/)

### 🧑‍💻 Author

Aditya
