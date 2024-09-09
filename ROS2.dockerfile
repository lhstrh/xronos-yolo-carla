FROM pytorch/pytorch:2.2.2-cuda12.1-cudnn8-runtime
RUN set -x && apt-get update \
    && apt-get install -y nano gnupg wget sudo make build-essential tmux curl
SHELL ["/bin/bash", "-c"]

# Create virtual environment
RUN pip3 install virtualenv
RUN virtualenv -p python3 ./venv \
    && source ./venv/bin/activate \
    && touch ./venv/COLCON_IGNORE \ 
    && echo 'source ./venv/bin/activate' >> ~/.bashrc 

# Install Python Packages
RUN source ./venv/bin/activate && pip3 install --no-cache-dir carla hydra-core pandas opencv-python-headless gym==0.25 agents tensorflow_probability tf_keras stable_baselines3 wandb shapely 
COPY scripts/test_gpu.py /workspace/
RUN apt-get update && apt-get install -y ffmpeg libsm6 libxext6
RUN source ./venv/bin/activate && pip3 install ultralytics && python test_gpu.py
ADD CARLA_0.9.15_PythonAPI.tar.gz /workspace/CARLA_0.9.15/

# Install ROS2
RUN apt install -y software-properties-common \
    && add-apt-repository -y universe \
    && apt update \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
ARG DEBIAN_FRONTEND=noninteractive
RUN apt install locales -y \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
ARG LANG=en_US.UTF-8
RUN apt update && apt upgrade -y
RUN apt install -y ros-humble-ros-base 
RUN apt install -y python3-colcon-common-extensions
RUN source ./venv/bin/activate && pip3 install empy==3.3.4 catkin_pkg lark
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Install ffmpeg 4.0.3
RUN wget https://www.johnvansickle.com/ffmpeg/old-releases/ffmpeg-4.0.3-64bit-static.tar.xz && tar -xvf ffmpeg-4.0.3-64bit-static.tar.xz && cd ffmpeg-4.0.3-64bit-static && sudo mv ffmpeg /usr/local/bin/ && rm /opt/conda/bin/ffmpeg && rm -r /workspace/ffmpeg-4.0.3-64bit-static && rm /workspace/ffmpeg-4.0.3-64bit-static.tar.xz

# Copy Model Weights
COPY models/ckpt_11833344.pth /workspace/ckpt/ckpt_11833344.pth
COPY models/yolov5n.pt /workspace/logs/
COPY scripts /workspace
COPY carla /workspace/src/carla_sim/carla_sim

# Setup Environment Variables 
ENV MILE_ROOT=/workspace/
ENV PATH=/workspace/lingua-franca/bin:/workspace/lingua-franca/build/install/lf-cli/bin:$PATH
ENV PYTHONPATH=/workspace/src/carla_sim/carla_sim/:/workspace/CARLA_0.9.15/PythonAPI/carla/:/workspace/venv/lib/python3.10/site-packages

# Copy ROS2 code
COPY ROS2 /workspace/
COPY scripts/time_logger.py /workspace/carla_sim/carla_sim/time_logger.py

RUN python modify_config.py --carla_ip="10.41.14.160" --server_controller_url="http://10.41.14.160:2010" --config_file="./src/carla_sim/carla_sim/config/carla.yaml"
