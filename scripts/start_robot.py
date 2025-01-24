import rclpy
import sys
from robot_module.robot_type1 import RobotType1
from robot_module.robot_type2 import RobotType2
from robot_module.robot_type3 import RobotType3

def main():
    if len(sys.argv) < 2:
        print("Usage: python scripts/start_robot.py <robot_type>")
        return
    
    robot_type = sys.argv[1]
    rclpy.init()

    if robot_type == "robot_type1":
        node = RobotType1()
    elif robot_type == "robot_type2":
        node = RobotType2()
    elif robot_type == "robot_type3":
        node = RobotType3()
    else:
        print("Invalid robot type")
        return

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
