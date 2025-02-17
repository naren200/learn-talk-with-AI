# Stage 1: Builder environment
FROM ros:iron-ros-base AS builder

# Install build dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    git \
    make \
    libsdl2-dev \
    ffmpeg \
    wget \
    software-properties-common \
    g++ \
    curl \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install CUDA
RUN wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb \
    && dpkg -i cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb \
    && cp /var/cuda-repo-ubuntu2204-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends cuda-11-8 \
    && rm -rf /var/lib/apt/lists/* \
    && rm cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb

ENV CUDA_HOME=/usr/local/cuda-11.8 \
    PATH="/usr/local/cuda-11.8/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH}"

# Build whisper.cpp
WORKDIR /root/ros2_ws/src/
RUN git clone https://github.com/ggerganov/whisper.cpp.git -b v1.7.4 --depth 1
WORKDIR /root/ros2_ws/src/whisper.cpp
RUN bash ./models/download-ggml-model.sh small.en && \
bash ./models/download-ggml-model.sh large-v1

RUN cmake -B build \
    -DWHISPER_SDL2=ON \
    -DGGML_CUDA=1 \
    -DCUDAToolkit_ROOT=/usr/local/cuda-11.8 && \
    cmake --build build -j --config Release && \
    make -j$(nproc) 

# Stage 2: Runtime environment
FROM ros:iron-ros-base

# Setup ROS2 environment
RUN echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc

# Install ALL dependencies in one layer
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    libsdl2-2.0-0 \
    ffmpeg \
    pulseaudio \
    pulseaudio-utils \
    libasound2-plugins \
    alsa-utils \
    make \
    g++ \
    sox \
    libasound2-dev \
    libsdl2-dev \
    python3-dev \
    python3-pip \
    python3-pybind11 \
    ros-humble-pybind11-vendor \
    && rm -rf /var/lib/apt/lists/*

# Copy CUDA runtime
COPY --from=builder /usr/local/cuda-11.8/lib64 /usr/local/cuda-11.8/lib64/
# COPY --from=builder /usr/local/cuda-11.8/ /usr/local/cuda-11.8/

# Copy whisper.cpp artifacts
COPY --from=builder /root/ros2_ws/src/whisper.cpp/ /usr/local/src/whisper.cpp/

# Environment configuration
ENV CUDA_HOME=/usr/local/cuda-11.8 \
    PATH="/usr/local/cuda-11.8/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH}"

# Install Ollama 
RUN apt-get update && \
    apt-get install -y curl && \
    curl -fsSL https://ollama.com/install.sh | bash && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create necessary directories
RUN mkdir -p /root/.ollama

# Volume for Ollama models
VOLUME /root/.ollama

# Environment variables for Ollama
ENV PATH="/usr/local/bin:${PATH}"
ENV WHISPER_MODEL="ggml-small.en.bin"


# # Speeching Module
RUN git clone https://github.com/myshell-ai/MeloTTS.git && \
    cd MeloTTS && \
    pip install -e . && \ 
    cd .. && \
    rm -rf MeloTTS/ && \
    pip install git+https://github.com/myshell-ai/MeloTTS.git && \
    python3 -m unidic download && \
    python3 -c "import nltk; nltk.download('averaged_perceptron_tagger_eng')"


# Final setup
RUN useradd -m -G audio pulseuser
RUN mkdir /root/ros2_ws/src/ -p
WORKDIR /root/ros2_ws/
CMD ["tail", "-f", "/dev/null"]
