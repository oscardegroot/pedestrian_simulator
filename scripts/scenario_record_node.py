from turtle import position
import rclpy
from rclpy.node import Node
import math
import os, sys

from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseStamped

def quaternion_to_angle(pose):
    ysqr = pose.orientation.y * pose.orientation.y
    t3 = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
    t4 = 1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z)
    return math.atan2(t3, t4)


class Pedestrian:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

class ScenarioRecorder:

    pedestrians = []

    def __init__(self) -> None:
        pass
    

    def add_pedestrian(self, start, goal):
        self.pedestrians.append(Pedestrian(start, goal))

    def to_file(self, file_name):
        path = os.path.dirname(os.path.abspath(__file__))
        file_path = path + "/../scenarios/" + file_name
        # return

        with open(file_path, 'w') as file:
            file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            file.write('<tag type="gaussian"/>\n')
            file.write('<tag type="velocity" value="1.4"/>\n')
            for ped in self.pedestrians:
                file.write('<pedestrian>\n')
                file.write(f'\t<start x="{ped.start.x:.4f}" y="{ped.start.y:.4f}"/>\n')
                file.write('\t<path>\n')
                file.write(f'\t\t<point x="{ped.goal.x:.4f}" y="{ped.goal.y:.4f}"/>\n')
                file.write('\t</path>\n')
                file.write('</pedestrian>\n')
            
            file.close()
        print(f"Scenario file saved at {file_path}")


class MinimalSubscriber(Node):

    def __init__(self, file_name, divider="autoware"):
        super().__init__('pedestrian_scenario_recorder')

        self.file_name = divider + "/" + file_name + ".xml"
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.listener_callback,
            1)        
        self.write_subscription = self.create_subscription(
            PoseStamped,
            '/planning/mission_planning/goal',
            self.write_callback,
            1)
        self.pose_subscription  # prevent unused variable warning
        self.write_subscription  # prevent unused variable warning
        self.recorder = ScenarioRecorder()

    def write_callback(self, msg):
        print("Writing a pedestrian simulation from retrieved poses")
        self.recorder.to_file(self.file_name)

    def listener_callback(self, msg):
        pos = msg.pose.pose.position
        # orientation = msg.pose.pose.orientation
        # orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = quaternion_to_angle(msg.pose.pose)
        end = Point()
        end.x = pos.x + math.cos(yaw) * 1.
        end.y = pos.y + math.sin(yaw) * 1.

        self.get_logger().info(f"Pedestrian [Start: {pos.x}, {pos.y} | Goal: {end.x}, {end.y}]")
        self.recorder.add_pedestrian(pos, end)


   
def main(args=None):
    rclpy.init(args=args)

    print("Pedestrian scenario recorder started. Publish on /initialpose to add pedestrians")
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    else:
        file_name = "temp"

    minimal_subscriber = MinimalSubscriber(file_name)

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()