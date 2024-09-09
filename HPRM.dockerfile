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
RUN sudo apt-get update && apt-get install -y ffmpeg libsm6 libxext6 git unzip zip
RUN source ./venv/bin/activate && pip3 install ultralytics && python test_gpu.py
ADD CARLA_0.9.15_PythonAPI.tar.gz /workspace/CARLA_0.9.15/
RUN apt-get install -y openjdk-17-jdk openjdk-17-jre

# Install ffmpeg 4.0.3
RUN wget https://www.johnvansickle.com/ffmpeg/old-releases/ffmpeg-4.0.3-64bit-static.tar.xz && tar -xvf ffmpeg-4.0.3-64bit-static.tar.xz && cd ffmpeg-4.0.3-64bit-static && sudo mv ffmpeg /usr/local/bin/ && rm /opt/conda/bin/ffmpeg && rm -r /workspace/ffmpeg-4.0.3-64bit-static && rm /workspace/ffmpeg-4.0.3-64bit-static.tar.xz

# Install Lingua Franca HPRM Version
RUN echo 2
RUN curl -Ls https://install.lf-lang.org | bash -s cli

# Install RTI
RUN git clone https://github.com/lf-lang/reactor-c.git \
    && cd reactor-c/core/federated/RTI \
    && mkdir build && cd build \
    && cmake ../ \
    && make \
    && sudo make install

RUN echo 1
# Copy Code & Model Weights
COPY carla /workspace/
COPY scripts /workspace/
COPY serializer /workspace/serializer
COPY models/ckpt_11833344.pth /workspace/ckpt/ckpt_11833344.pth
COPY models/yolov5n.pt /workspace/logs/

# Install Out-of-band Serializer
RUN source ./venv/bin/activate && cd /workspace/serializer && pip3 install -e . && pip install -e . 

# Setup Environment Variables 
ENV MILE_ROOT=/workspace/
ENV PATH=/root/.local/bin:/workspace/lingua-franca/bin:/workspace/lingua-franca/build/install/lf-cli/bin:$PATH
ENV PYTHONPATH=/workspace/CARLA_0.9.15/PythonAPI/carla/

# Copy HPRM CARLA example code
COPY HPRM /workspace/

RUN python modify_config.py --carla_ip="10.41.14.160" --server_controller_url="http://10.41.14.160:2010" --config_file="./config/carla.yaml" && echo 'tmux new -d -s plasma plasma_store -m 4000000000 -s /tmp/plasma' >> ~/.bashrc 