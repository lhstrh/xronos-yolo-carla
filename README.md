# HPRM: High-Performance Robotic Middleware for Intelligent Autonomous Systems

[![Website](https://img.shields.io/badge/Website-HPRM-Red)](https://depetrol.github.io/HPRM/)
[![Github](https://img.shields.io/badge/Github-HPRM-orange)](https://github.com/Depetrol/HPRM)
[![Docker](https://img.shields.io/badge/DockerHub-HPRM-blue)](https://hub.docker.com/repository/docker/depetrol/hprm/general)
[![Docker](https://img.shields.io/badge/DockerHub-ROS2-blue)](https://hub.docker.com/repository/docker/depetrol/ros2/general)
[![Docker](https://img.shields.io/badge/DockerHub-Xronos-blue)](https://hub.docker.com/repository/docker/depetrol/xronos/general)

This repo contains the implementation of the CARLA autonomous driving benchmark for paper HPRM: High-Performance Robotic Middleware for Intelligent Autonomous Systems. All code has been containerized with Docker for reproducability.

## 🏛️ Repo Structure

* `/HPRM`: Code for building HPRM docker image.
* `/ROS2`: Code for building ROS2 docker image.
* `/carla_server_controller`: Because Carla uses a server-client architecture, and `carla_server_controller` serves as a utility tool to control the startup and shutdown of the carla server.
* `/models`: Model weights
  * `ckpt_11833344.pth` for the autonomous driving algorithm.
  * `yolov5n.pt` for the YOLO object detection algorithm.
* `/CARLA_0.9.15_PythonAPI.tar.gz` for Carla PythonAPI bindings.

## 🚀 HPRM For Your Application

1. Install latest version of Lingua Franca

   ```bash
   curl -Ls https://install.lf-lang.org | bash -s cli
   ```
2. Install and use desired serializer by following the instructions [here](https://github.com/Depetrol/Serializers).

## 🔄 Reproduce CARLA Benchmark in HPRM and ROS2

### CARLA Simulator Server and Controller Setup

1. Install the CARLA simulator version `0.9.15`. Because CARLA uses a server-client architecture, CARLA server can be installed anywhere (windows/linux) as long as a network connection can be established between the CARLA simulator and the Docker container of HPRM or ROS2. The following setup assumes that CARLA is installed on the host machine and a Docker container is spawned from the host, but other setups can be configured as well by changing the Docker network configurations.
2. Copy the CARLA server controller in `carla_server_controller`. This is a python script that starts a web server that receives command from the Docker container to orchestrate startup / shutdown of the CARLA server. This script should be installed on the same machine as the CARLA simulator.
3. Install Flask: `pip3 install flask`
4. Run the server controller: `python3 carla_server_controller.py --carlapath=<path to CARLA executable>"`. If CARLA is installed on Windows, modify `run_controller.bat` with your own CARLA install position and double click on the `.bat` file to run the server controller.
5. When the script starts up, it will output something like:

   ```
    * Running on http://127.0.0.1:2010
    * Running on http://198.18.0.1:2010
   ```

   copy the second URL as it will be used to connect the Docker container to the server controller. The following setup will use http://198.18.0.1:2010 as the server URL, please **substitute with your network URL**.

### HPRM Setup

1. Pull Docker image from Docker Hub: `docker pull depetrol/hprm`
2. Run a Docker container from the image:

   ```
   docker run -it --gpus=all --shm-size 12g  --net=host depetrol/hprm
   ```

   This will do the following:

   - Start the bash shell in interactive mode, with access to all GPUs(if the drivers are configured correctly).
   - Allow the max memory used by the docker image to be 12 GB.
   - Share the network stack of the host with the container. This allows the container to connect to the CARLA server and the server controller.
3. Check if the Docker container can connect to the CARLA server controller with provided script: `python server_test.py --server_controller_url http://198.18.0.1:2010`
4. Modify the config file with script to connect to your CARLA environment: `python modify_config.py --carla_ip="198.18.0.1" --server_controller_url="http://198.18.0.1:2010" --config_file="./config/carla.yaml"`
5. Compile and run HPRM:

   - centralized version: `lfc carla_centralized.lf && ./bin/carla_centralized`
   - decentralized version: `lfc carla_decentralized.lf && ./bin/carla_decentralized`

### ROS2 Setup

1. Pull Docker image from Docker Hub: `docker pull depetrol/ros2`
2. Run a Docker container from the image:

   ```
   docker run -it --gpus=all --shm-size 12g  --net=host depetrol/ros2
   ```
3. Check if the Docker container can connect to the CARLA server controller with provided script: `python server_test.py --server_controller_url http://198.18.0.1:2010`
4. Modify the config file with script: `python modify_config.py --carla_ip="198.18.0.1" --server_controller_url="http://198.18.0.1:2010" --config_file="./src/carla_sim/carla_sim/config/carla.yaml"`
5. Build the ROS2 implementation: `colcon build --symlink-install && python disable_checker.py`
6. Because ROS2 needs a new terminal to start each node, we start tmux so we can start multiple windows: `tmux`
7. Create three windows with `tmux split-window -h`, and nevigate between them with `Ctrl B` then `left arrow / right arrow` key.
8. Start and run ROS2 with the following steps. The startup order matters in the case: the CARLA node must be started last because it will start the process of publishing messages to the other two nodes. After starting each node, wait until the node outputs `=== ... Ready ===` which means the node is ready to accept messages.

   1. Start the YOLO object detection node: `source install/setup.bash && ros2 run carla_sim yolo`
   2. Start the PPO autonomous driving node: `source install/setup.bash && ros2 run carla_sim agent`
   3. Start the FUSION node: `source install/setup.bash && ros2 run carla_sim fusion`
   4. Start the CARLA simulation node: `source install/setup.bash && ros2 run carla_sim carla`

### Xronos Setup

1. Pull Docker image from Docker Hub: `docker pull depetrol/xronos`
2. Run a Docker container from the image:

   ```
   docker run -it --gpus=all --shm-size 12g  --net=host depetrol/xronos
   ```
3. Check if the Docker container can connect to the CARLA server controller with provided script: `python server_test.py --server_controller_url http://198.18.0.1:2010`
4. Modify the config file with script to connect to your CARLA environment: `python modify_config.py --carla_ip="198.18.0.1" --server_controller_url="http://198.18.0.1:2010" --config_file="./config/carla.yaml"`
5. Run Benchmark with Xronos: `python xronos_carla.py`

## 🛠️ Build Docker Images

This section is for building the Docker image from source code.

First download `CARLA_0.9.15_PythonAPI.tar.gz` from the Release section of this Github repository and put them in the repo root directory(alongside this README.md) for Carla PythonAPI bindings.

* HPRM: `docker build -f HPRM.dockerfile -t depetrol/hprm:latest .`
* ROS2: `docker build -f ROS2.dockerfile -t depetrol/ros2:latest .`

## 🙌 Credits

Thanks to the authors of [Model-Based Imitation Learning for Urban Driving](https://github.com/wayveai/mile) for providing a implementation of the PPO autonomous driving algorithm and corresponding configuration code. We modifyed the code to work with CARLA 0.9.15.

Thanks to the authors of [End-to-End Urban Driving by Imitating a Reinforcement Learning Coach](https://github.com/zhejz/carla-roach) for providing a gym wrapper around CARLA making it easy to use, and the RL expert PPO autonomous driving algorithm.
