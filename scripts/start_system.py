import subprocess
import yaml

CONFIG_FILE = "config/config1.yaml"

def load_config():
    with open(CONFIG_FILE, "r") as file:
        return yaml.safe_load(file)

def start_nodes(config):
    processes = []
    for node in config["nodes"]:
        module = node["module"].split("_")[0]  # "camera", "image_processing", "robot"
        node_name = node["name"]
        process = subprocess.Popen(["python", f"scripts/start_{module}.py", node_name])
        processes.append(process)

    # 모든 노드가 종료될 때까지 대기
    for process in processes:
        process.wait()

if __name__ == "__main__":
    config = load_config()
    start_nodes(config)
