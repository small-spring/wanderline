# Python 3.12ベースの公式イメージを使用
FROM python:3.12-slim

# ROS2環境変数を設定
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# 作業ディレクトリを設定
WORKDIR /workspace

# システムの依存関係をインストール (Wanderline + ROS2)
RUN apt-get update && apt-get install -y \
    # Wanderline dependencies
    git \
    curl \
    build-essential \
    libopencv-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libgdal-dev \
    # ROS2 dependencies
    software-properties-common \
    lsb-release \
    wget \
    gnupg2 \
    # GUI support for Gazebo
    x11-apps \
    mesa-utils \
    # Python dev tools
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humble リポジトリを追加
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble + Gazebo + Universal Robots をインストール
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-* \
    ros-humble-moveit \
    ros-humble-ur-robot-driver \
    ros-humble-ur-description \
    ros-humble-joint-state-publisher-gui \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# rosdep を初期化
RUN rosdep init && rosdep update

# uvをインストール（高速Pythonパッケージマネージャー）
RUN pip install uv

# プロジェクトファイルをコピー
COPY pyproject.toml ./
COPY uv.lock* ./

# 仮想環境を作成し、依存関係をインストール
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN /opt/venv/bin/pip install --upgrade pip
RUN uv venv /opt/venv --system-site-packages
RUN uv sync --frozen || uv sync || pip install torch opencv-python numpy matplotlib jupyter notebook ipywidgets

# Jupyter Notebookもインストール（開発用）
RUN /opt/venv/bin/pip install jupyter notebook ipywidgets matplotlib

# ROS2環境をbashrcに追加
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export DISPLAY=\${DISPLAY:-:0}" >> ~/.bashrc

# ROS2ワークスペースを作成
RUN mkdir -p /workspace/robot_ws/src

# 開発用のポートを公開 (ROS2用も追加)
EXPOSE 8888 8000 11311

# エントリーポイント (ROS2環境をソース)
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && bash"]
