FROM ros:humble

# 작업 디렉토리 설정
WORKDIR /ros2_ws

# 프로젝트 전체 복사
COPY . /ros2_ws

# 시스템 패키지 업데이트 및 필수 패키지 설치
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ros-base \
    && apt clean && rm -rf /var/lib/apt/lists/*

# Python 의존성 설치
RUN pip install --no-cache-dir -r requirements.txt

# ROS 2 패키지 빌드
RUN . /opt/ros/humble/setup.sh && colcon build

# 실행 스크립트 복사
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 컨테이너 실행 시 entrypoint.sh 실행
ENTRYPOINT ["/entrypoint.sh"]
