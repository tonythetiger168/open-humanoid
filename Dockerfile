FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-matplotlib \
    libgl1-mesa-glx \
    libglib2.0-0 \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Install PyBullet (optional, for simulation)
RUN pip3 install --no-cache-dir pybullet

# Copy project
WORKDIR /workspace/open_humanoid
COPY . /workspace/open_humanoid/

# Set environment
ENV PYTHONPATH=/workspace/open_humanoid:$PYTHONPATH

# Default command
CMD ["python3", "control/main_controller.py"]
