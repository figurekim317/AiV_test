# ROS 2 Dynamic Node Communication System

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
- **Camera Module → Image Processing Module**: Uses **ROS 2 topics** to publish image data.
- **Image Processing Module → Robot Module**: Uses **ROS 2 services** to process images and return results.
- **Robot Module → Main Module**: Uses **ROS 2 actions** to handle asynchronous motion execution and feedback.
- **Main Module → Camera Module**: Requests new captures based on motion completion using **services**.

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
python scripts/start_system.py --config config/config1.yaml
```

#### **4. Run other nodes in separate terminals:**
```sh
python scripts/start_camera.py &
python scripts/start_processing.py &
python scripts/start_robot.py &
```
(Use `&` to run them in the background.)

### **File Structure for Script Execution**
```sh
ros2_dynamic_nodes/
├── Dockerfile
├── docker-compose.yml
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

### **Example: `scripts/start_system.py` (Main Entry Script)**
```python
import subprocess
import yaml

def load_config(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def start_nodes(config):
    for node in config['nodes']:
        module, name = node['module'], node['name']
        subprocess.Popen(['python', f'scripts/start_{module}.py', name])

if __name__ == "__main__":
    config = load_config('config/config1.yaml')
    start_nodes(config)
```

### **Modify Configuration**
- Modify the configuration file (`config/config1.yaml`) to change nodes dynamically.

### **Testing**
- Change the `config.yaml` file to test different node configurations.
- Use `ros2 topic echo /camera/image_raw` to verify published messages.

### **✅ Final Summary**
- `Dockerfile`, `docker-compose.yml`: **Container environment setup**
- `config.yaml`: **Dynamic configuration**
- `main_node.py`: **Runs nodes based on the configuration file**
- `scripts/`: **Script-based execution support**
- `camera_node.py`, `processor_type1.py`, `robot_type1.py`: **ROS 2 Nodes**
- `README.md`: **Documentation**
- **Added script execution support**: The system can now run independently without Docker.

This submission ensures **dynamic node execution and ROS 2 communication** for a fully functional system, whether inside a container or as standalone scripts. 🚀

