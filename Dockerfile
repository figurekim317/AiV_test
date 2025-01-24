FROM ros:humble

WORKDIR /ros2_ws

COPY . /ros2_ws

RUN apt update && apt install -y python3-pip
RUN pip install -r requirements.txt

# ROS 2 패키지 빌드
RUN . /opt/ros/humble/setup.sh && colcon build

ENTRYPOINT ["ros2", "launch", "launch", "main_launch.py"]
