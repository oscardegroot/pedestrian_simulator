#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Quaternion
from std_srvs.srv import Empty, EmptyResponse
from pedestrian_simulator.srv import SpawnWall, SpawnWallResponse


def create_wall_sdf(size, name, color="0.0 0.0 0.0 1.0"):
    # Generate an SDF string for a wall with the specified size and color
    sdf = f"""
    <sdf version="1.6">
      <model name="{name}">
        <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>{size[0]} {size[1]} {size[2]}</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>{size[0]} {size[1]} {size[2]}</size>
              </box>
            </geometry>
            <material>
              <ambient>{color}</ambient>
              <diffuse>{color}</diffuse>
              <specular>0.1 0.1 0.1 1.0</specular>
              <emissive>0.0 0.0 0.0 1.0</emissive>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """
    return sdf

def handle_spawn_wall(req):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    try:
        # Create a handle to the service
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        # Prepare the request for the wall model
        request = SpawnModelRequest()

        request.model_name = req.name if req.name else "wall"
        request.model_xml = create_wall_sdf(req.size, request.model_name)
        request.robot_namespace = ""
        
        # Set the initial pose of the wall
        request.initial_pose.position.x = req.position[0]
        request.initial_pose.position.y = req.position[1]
        request.initial_pose.position.z = req.position[2]

        # Set the orientation of the wall
        request.initial_pose.orientation.x = req.orientation[0]
        request.initial_pose.orientation.y = req.orientation[1]
        request.initial_pose.orientation.z = req.orientation[2]
        request.initial_pose.orientation.w = req.orientation[3]

        # Specify the Gazebo world
        request.reference_frame = "world"

        # Call the service to spawn the wall
        spawn_model_prox(request)
        # rospy.loginfo(f"Wall '{request.model_name}' spawned successfully!")

        return SpawnWallResponse(success=True, message=f"Spawned wall at {{x = {req.position[0]:.1f}, y = {req.position[1]:.1f}}}.")


    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return SpawnWallResponse(success=False, message=str(e))

def spawn_wall_server():
    rospy.init_node('spawn_wall_server')

    # Define the service and the callback function
    s = rospy.Service('spawn_wall', SpawnWall, handle_spawn_wall)
    rospy.loginfo("Ready to spawn walls.")
    
    # Keep the service alive
    rospy.spin()

if __name__ == "__main__":
    try:
        spawn_wall_server()
    except rospy.ROSInterruptException:
        pass