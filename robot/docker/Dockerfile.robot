# Robot Simulation Docker Environment
# Uses existing ROS2 image with Gazebo support

FROM tiryoh/ros2-desktop-vnc:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONPATH=/workspace

WORKDIR /workspace

# Install additional Python dependencies and ROS2 packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages (separate step to avoid conflicts)
RUN apt-get update && apt-get install -y \
    ros-humble-ur-description \
    ros-humble-ur-robot-driver \
    && rm -rf /var/lib/apt/lists/*

# Install Python package manager
RUN pip install uv

# Copy project files
COPY pyproject.toml uv.lock* ./

# Install Python dependencies
RUN uv sync

# Install additional development tools
RUN uv add jupyter notebook ipywidgets matplotlib

# Note: Using base image's default websockify configuration (port 80)