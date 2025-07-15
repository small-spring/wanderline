# Robot Simulation Docker Environment
# Uses existing ROS2 image with Gazebo support

FROM tiryoh/ros2-desktop-vnc:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONPATH=/workspace

WORKDIR /workspace

# Install additional Python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python package manager
RUN pip install uv

# Copy project files
COPY pyproject.toml uv.lock* ./

# Install Python dependencies
RUN uv sync

# Install additional development tools
RUN uv add jupyter notebook ipywidgets matplotlib

# Entry point with ROS2 environment
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && bash"]