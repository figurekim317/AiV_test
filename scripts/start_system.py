import subprocess
import yaml
import os
import signal

CONFIG_FILE = os.getenv("ROS_CONFIG_FILE", "config/config1.yaml")

def load_config():
    """Loads the configuration file and returns the list of nodes to launch."""
    try:
        with open(CONFIG_FILE, "r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading configuration file: {e}")
        return None

def start_nodes(config):
    """Starts the nodes defined in the configuration file."""
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

        # Wait for all nodes to complete execution
        for process in processes:
            process.wait()

    except KeyboardInterrupt:
        print("\nSystem interrupted. Shutting down nodes...")
    finally:
        for process in processes:
            process.send_signal(signal.SIGTERM)  # Send termination signal
        for process in processes:
            process.wait()  # Wait for processes to exit safely
        print("All nodes stopped successfully.")

if __name__ == "__main__":
    config = load_config()
    if config:
        start_nodes(config)
    else:
        print("Error: Invalid configuration. Exiting.")
