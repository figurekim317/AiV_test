FROM ros:humble

# Set working directory
WORKDIR /ros2_ws

# Copy project files into the container
COPY . /ros2_ws

# Update system and install required dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ros-base \
    && apt clean && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Build ROS 2 packages
RUN . /opt/ros/humble/setup.sh && colcon build

# Copy and set up entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Execute the entrypoint script on container startup
ENTRYPOINT ["/entrypoint.sh"]
