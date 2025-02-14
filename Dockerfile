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
    libasound2-dev \
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
ENV OLLAMA_HOST=0.0.0.0:11434
ENV PATH="/usr/local/bin:${PATH}"
ENV MODEL="ggml-small.en.bin"

# Expose Ollama port
EXPOSE 11434

# Audio configuration
RUN echo 'pcm.!default { \n\
        type plug \n\
        slave { \n\
            pcm "hw:0,0" \n\
            rate 16000 \n\
            buffer_size 16000 \n\
            period_size 8000 \n\
        } \n\
    }\n\
    defaults.pcm.rate_converter "speexrate_best"\n\
    ' > /etc/asound.conf

# Final setup
RUN useradd -m -G audio pulseuser
RUN mkdir /root/ros2_ws/src/ -p
WORKDIR /root/ros2_ws/
CMD ["tail", "-f", "/dev/null"]
