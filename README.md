!! Work is on route !!
# Talk and learn with AI: Seamless interactions


## Overview
This project enables seamless interaction with AI through Docker and ROS2. The setup includes developer mode, allowing developers to attach directly to running Docker containers for debugging, testing, and configuration without rebuilding.

## Features
- **Developer Mode:** Attach to running Docker containers.
- **Seamless AI Interactions:** Run individual nodes and launch files.
- **Log Inspection & File Modification:** Directly access and modify container files.
- **ROS2 Integration:** Easily build and launch ROS2 packages.

## Prerequisites
Ensure your system meets the following requirements:
- Ubuntu 22.04 or higher
- Docker and Docker Compose
- ROS2 (tested on Iron)

### Dependencies Installation Guide

#### 1. Install Docker on Ubuntu 22.04
Follow this guide to install Docker:  
[How to Install and Use Docker on Ubuntu 22.04](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04)

#### 2. Perform Post-Installation Steps for Docker
Ensure you complete the post-installation steps:  
[Post-Installation Steps for Docker on Linux](https://docs.docker.com/engine/install/linux-postinstall/)

#### 3. Install NVIDIA Container Toolkit (For GPU Users)
If you're using a GPU, install the NVIDIA Container Toolkit:  
[NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

#### 4. Install Docker Compose on Ubuntu 22.04
Set up Docker Compose using this guide:  
[How to Install and Use Docker Compose on Ubuntu 22.04](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-compose-on-ubuntu-22-04)

## Developer Mode Setup
Enable developer mode by setting the environment variable:
```bash
export DEVELOPER=True
```

### Start Docker in Developer Mode
Run the following command to start the Docker container and attach it automatically:
```bash
./start_docker.sh --developer=true
```

This allows you to:
- Run individual ROS2 nodes or launch files.
- Access and modify container files.
- Inspect logs and diagnose issues.
- Test configurations without rebuilding.

## Running the ROS2 Launch File
To build and launch the ROS2 package:
```bash
colcon build && source install/setup.bash && ros2 launch src/launch/conversation_launch.py
```

## Work in Progress
This project is actively being developed. Stay tuned for updates!


### Speak node: ChatTTS
With GPU access working; python script though


### Think node: ollama with GPU (Pending: OpenAI, XAI, Deepseek integrations)
A minimal code structure is uploaded


### Listen node: whisper-streaming with GPU dockerfile 
A minimal code structure is uploaded


Next todo: dockercompose multiple container integrations