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
    random_points = []

    def __init__(self) -> None:
        pass
    
    def add_random_point(self, start):
        self.random_points.append(start)

    def add_pedestrian(self, start, goal):
        self.pedestrians.append(Pedestrian(start, goal))

    def to_file(self, file_name, random=False):
        path = os.path.dirname(os.path.abspath(__file__))
        file_path = path + "/../scenarios/" + file_name
        # return

        type = "social" if random else "gaussian"

        with open(file_path, 'w') as file:
            file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            file.write(f'<tag type="{type}"/>\n')
            file.write('<tag type="velocity" value="1.4"/>\n')

            if not random:
                for ped in self.pedestrians:
                    file.write('<pedestrian>\n')
                    file.write(f'\t<start x="{ped.start.x:.4f}" y="{ped.start.y:.4f}"/>\n')
                    file.write('\t<path>\n')
                    file.write(f'\t\t<point x="{ped.goal.x:.4f}" y="{ped.goal.y:.4f}"/>\n')
                    file.write('\t</path>\n')
                    file.write('</pedestrian>\n')
            else:
                min_x = min(self.random_points, key=lambda x: x.x).x
                max_x = max(self.random_points, key=lambda x: x.x).x
                min_y = min(self.random_points, key=lambda x: x.y).y
                max_y = max(self.random_points, key=lambda x: x.y).y
                file.write('<random>\n')
                file.write(f'\t<range_x min="{min_x}" max="{max_x}"/>\n')
                file.write(f'\t<range_y min="{min_y}" max="{max_y}"/>\n')
                file.write('\t<range_v min="1.14" max="1.66"/>\n')
                file.write('\t<goal_inflation value="50.0"/>\n')
                file.write('</random>\n')

                file.write('<random_pedestrians value="4"/>\n')

        print(f"Scenario file saved at {file_path}")


class MinimalSubscriber(Node):

    def __init__(self, file_name, use_random_area, divider="autoware"):
        super().__init__('pedestrian_scenario_recorder')

        self.file_name = divider + "/" + file_name + ".xml"
        self.use_random_area = use_random_area
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

        if not self.use_random_area:
            self.get_logger().info(f"Pedestrian [Start: {pos.x}, {pos.y} | Goal: {end.x}, {end.y}]")
            self.recorder.add_pedestrian(pos, end)
        else:
            self.recorder.add_random_point(pos)
            if len(self.recorder.random_points) >= 2:
                self.recorder.to_file(self.file_name, random=True)
                self.recorder.random_points = []


   
def main(args=None):
    rclpy.init(args=args)

    print("Pedestrian scenario recorder started. Publish on /initialpose to add pedestrians")
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    else:
        file_name = "temp"

    if len(sys.argv) > 2:
        use_random_area = sys.argv[2] == "True"
    else:
        use_random_area = False

    minimal_subscriber = MinimalSubscriber(file_name, use_random_area)

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()