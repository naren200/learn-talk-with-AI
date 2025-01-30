# FROM anibali/pytorch:2.0.1-cuda11.8
FROM ubuntu:22.04

USER root 

# Install base dependencies with proper package list formatting
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git \
    make \
    build-essential \
    libsdl2-dev \
    ffmpeg \
    wget \
    software-properties-common \
    alsa-base \
    alsa-utils \
    libasound2-dev \
    g++ \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y apt-utils

RUN wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb \
    && dpkg -i cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb \
    && cp /var/cuda-repo-ubuntu2204-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y cuda-11-8 \
    && rm -rf /var/lib/apt/lists/* \
    && rm cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb


ENV CUDA_HOME=/usr/local/cuda-11.8 \
    PATH="/usr/local/cuda-11.8/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:${LD_LIBRARY_PATH}"


# Clone whisper.cpp
WORKDIR /usr/local/src
RUN git clone https://github.com/ggerganov/whisper.cpp.git -b v1.7.4 --depth 1


# Build with explicit CUDA configuration
WORKDIR /usr/local/src/whisper.cpp
RUN bash ./models/download-ggml-model.sh small.en && \
    cmake -B build \
        -DWHISPER_SDL2=ON \
        -DGGML_CUDA=1 \
        -DCUDAToolkit_ROOT=/usr/local/cuda-11.8 && \
    cmake --build build -j --config Release && \
    make -j$(nproc)

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    pulseaudio \
    pulseaudio-utils \
    libasound2-plugins \
    && rm -rf /var/lib/apt/lists/*

# Add ALSA config with explicit sample rateh
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

# Create non-root user for PulseAudio
RUN useradd -m -G audio pulseuser

# Add audio group
RUN usermod -a -G audio root


# Configure PulseAudio
# COPY pulse-client.conf /etc/pulse/client.conf
# RUN mkdir -p /home/pulseuser/.config/pulse
# RUN chown -R pulseuser:pulseuser /home/pulseuser/.config/pulse

# Entrypoint script
# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh
# ENTRYPOINT ["/entrypoint.sh"]
# ENTRYPOINT []

# WORKDIR /usr/local/src/whisper.cpp/build/bin
CMD ["tail", "-f", "/dev/null"]
