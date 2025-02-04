# ROS 2 Dynamic Node Communication System

## **Problem: Developing a Flexible ROS 2 Node Communication System with Dynamic Node Configuration**

## **Background**
This project aims to develop a communication system using ROS 2 that enables interaction between camera, image processing, robot, and main modules. These modules will communicate through **topics, services, and actions**. Additionally, a custom ROS 2 interface package will be created, including messages, services, and actions.

### **Dynamic Node Loading in the Main Node**
- The `main_node` dynamically loads nodes by reading a `config.yaml` file at runtime.
- The system must remain operational even when module nodes are replaced.

#### **Example Configuration Files**
```yaml
# Example 1
nodes:
  - module: camera_module
    name: camera_type2
  - module: image_processing_module
    name: sensor_type1
  - module: robot_module
    name: robot_type2

# Example 2
nodes:
  - module: camera_module
    name: camera_type1
  - module: image_processing_module
    name: sensor_type2
  - module: robot_module
    name: robot_type3
```

## **Testing and Verification**
- Modify and test multiple configuration files to ensure the system operates as expected.

## **Additional Requirements**
This project does not require direct implementation; instead, it focuses on documenting the architecture and methodology via a markdown file in the GitHub repository.

- The proposed solution should support:
  - Multiple cameras, where each captured image undergoes different image processing.
  - Dynamic adjustment of robot motion scenarios based on sensor feedback.
  - Workflow modifications based on conditional logic.

#### **Example Use Cases**
1. **Multiple Camera Support**
    - Dynamic loading of camera configurations.
    - Routing of images to appropriate processing nodes.
    - Synchronization of data collected from multiple cameras.
2. **Adaptive Robot Motion**
    - Configurable motion sequences.
    - Real-time adjustments based on sensor feedback.
    - Dynamic selection of subsequent tasks after motion completion.
3. **Conditional Execution Flow**
    - Multi-stage trigger workflows.
    - Modifying execution order based on conditions.

## **Submission Requirements**
- Add **GitAivSW (sw@aiv.ai)** as a **contributor** to the **private GitHub repository**.
- The repository should include:
  - Source code files implementing the required features.
  - At least two example configuration files.
  - A detailed `README.md` explaining execution steps and test scenarios.

## **Grading Criteria**
- **Functionality**: Correct implementation of required features.
- **Readability & Maintainability**: Clean and structured code.
- **Extensibility**: Modular design for future enhancements.
- **Error Handling**: Robust mechanisms to handle edge cases.
- **Documentation & Testing**: Well-documented setup and verification steps.

---

## Overview
This document describes the design and architecture for a flexible ROS 2 node communication system that dynamically configures nodes based on a YAML configuration file. The system is implemented in a Docker environment and can also be run as standalone scripts. The system consists of five core modules:

- **Main Module** (Orchestrates the execution of nodes)
- **Camera Module** (Handles image capture from multiple camera types)
- **Image Processing Module** (Processes captured images with different algorithms)
- **Robot Module** (Executes motion sequences and interacts with sensors)
- **Interface Module** (Defines custom ROS 2 messages, services, and actions)

## System Design
The architecture follows a modular and scalable approach where different nodes within each module communicate using **topics, services, and actions**.

### **Node Communication**
- **Camera Module → Image Processing Module**: Uses **ROS 2 topics (`/camera/image_raw`)** to publish image data.
- **Image Processing Module → Robot Module**: Uses **ROS 2 services (`/image_processing/process_image`)** to process images and return results.
- **Robot Module → Main Module**: Uses **ROS 2 actions (`/robot/execute_motion`)** to handle asynchronous motion execution and feedback.
- **Main Module → Camera Module**: Requests new captures based on motion completion using **ROS 2 services (`/camera/capture_request`)**.

### **Dynamic Node Loading**
The main module reads a `config.yaml` file to determine which nodes should be instantiated at runtime. This allows easy reconfiguration without modifying the source code.

#### **Example Configuration File**
```yaml
nodes:
  - module: camera_module
    name: camera_type1
  - module: image_processing_module
    name: processor_type2
  - module: robot_module
    name: robot_type3
```

## Implementation Plan
### **Docker Environment**
The ROS 2 system runs inside Docker containers for easy deployment. The project includes:
- `Dockerfile` to set up the ROS 2 environment.
- `docker-compose.yml` to manage multiple containers.
- `entrypoint.sh` to dynamically configure and launch ROS 2 nodes.

### **Running as a Script (Without Docker)**
If you want to run the system without Docker, follow these steps:

#### **1. Ensure ROS 2 is installed and sourced:**
```sh
source /opt/ros/humble/setup.bash
```

#### **2. Install dependencies:**
```sh
pip install -r requirements.txt
```

#### **3. Run the main node:**
```sh
python scripts/start_system.py
```

#### **4. Run other nodes in separate terminals:**
```sh
python scripts/start_camera.py &
python scripts/start_processing.py &
python scripts/start_robot.py &
```
(Use `&` to run them in the background.)

### **Running with Docker**
#### **1. Build and run the container:**
```sh
docker-compose up --build
```
#### **2. Run with a specific configuration file:**
```sh
ROS_CONFIG_FILE=config/config2.yaml docker-compose up --build
```

### **Testing the System**
After launching the system, you can verify the communication between nodes using the following commands:

#### **1. Verify that the camera is publishing images:**
```sh
ros2 topic echo /camera/image_raw
```
#### **2. Verify that the image processing node is responding to service calls:**
```sh
ros2 service call /image_processing/process_image interface_module/srv/ProcessImage "{image_data: {data: []}}"
```
#### **3. Monitor robot motion execution feedback:**
```sh
ros2 action send_goal /robot/execute_motion interface_module/action/ExecuteMotion "{motion_type: 'walk'}"
```

### **Modify Configuration**
- Modify the configuration file (`config/config1.yaml`) to change nodes dynamically.
- Use the environment variable `ROS_CONFIG_FILE` to specify a different configuration file before starting the system.

### **File Structure for Execution**
```sh
ros2_dynamic_nodes/
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
├── README.md
├── config/
│   ├── config1.yaml
│   ├── config2.yaml
├── launch/
│   ├── main_launch.py
├── scripts/
│   ├── start_system.py
│   ├── start_camera.py
│   ├── start_processing.py
│   ├── start_robot.py
├── main_module/
│   ├── __init__.py
│   ├── main_node.py
├── camera_module/
│   ├── __init__.py
│   ├── camera_type1.py
│   ├── camera_type2.py
│   ├── camera_type3.py
├── image_processing_module/
│   ├── __init__.py
│   ├── processor_type1.py
│   ├── processor_type2.py
│   ├── processor_type3.py
├── robot_module/
│   ├── __init__.py
│   ├── robot_type1.py
│   ├── robot_type2.py
│   ├── robot_type3.py
├── interface_module/
│   ├── msg/
│   │   ├── ImageData.msg
│   ├── srv/
│   │   ├── ProcessImage.srv
│   ├── action/
│   │   ├── ExecuteMotion.action
```

### **✅ Final Summary**
- `Dockerfile`, `docker-compose.yml`, `entrypoint.sh`: **Container environment setup**
- `config.yaml`: **Dynamic configuration**
- `main_node.py`: **Runs nodes based on the configuration file**
- `scripts/`: **Script-based execution support**
- `camera_node.py`, `processor_type1.py`, `robot_type1.py`: **ROS 2 Nodes**
- `README.md`: **Documentation**
- **Expanded testing instructions**: Now includes `ros2 topic echo`, `ros2 service call`, and `ros2 action send_goal`.
- **Added dynamic configuration handling**: Users can change configurations using `ROS_CONFIG_FILE`.

This submission ensures **dynamic node execution and ROS 2 communication** for a fully functional system, whether inside a container or as standalone scripts. 🚀

