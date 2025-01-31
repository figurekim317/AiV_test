import subprocess
import yaml
import os
import signal

CONFIG_FILE = os.getenv("ROS_CONFIG_FILE", "config/config1.yaml")

def load_config():
    """설정 파일을 로드하여 실행할 노드 목록을 반환"""
    try:
        with open(CONFIG_FILE, "r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading configuration file: {e}")
        return None

def start_nodes(config):
    """설정 파일을 기반으로 각 노드를 실행"""
    processes = []
    try:
        for node in config["nodes"]:
            module = node["module"].split("_")[0]  # "camera", "image_processing", "robot"
            node_name = node["name"]

            script_path = f"scripts/start_{module}.py"
            if not os.path.exists(script_path):
                print(f"Error: {script_path} not found. Skipping {node_name}.")
                continue

            process = subprocess.Popen(["python", script_path, node_name])
            processes.append(process)
            print(f"Started {module} node: {node_name}")

        # 모든 노드가 종료될 때까지 대기
        for process in processes:
            process.wait()

    except KeyboardInterrupt:
        print("\nSystem interrupted. Shutting down nodes...")
    finally:
        for process in processes:
            process.send_signal(signal.SIGTERM)  # 안전한 종료 신호 전송
        for process in processes:
            process.wait()  # 모든 프로세스가 안전하게 종료될 때까지 대기
        print("All nodes stopped successfully.")

if __name__ == "__main__":
    config = load_config()
    if config:
        start_nodes(config)
    else:
        print("Error: Invalid configuration. Exiting.")
