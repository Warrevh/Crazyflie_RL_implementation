#!/usr/bin/env python

import sys
import time
import numpy as np
from pathlib import Path
from crazyflie_py import Crazyswarm
import csv
import os
import datetime

from stable_baselines3 import SAC

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import tf_transformations

from scipy.spatial.transform import Rotation as R

#from crazyflie_RL.droneposition import DronePosition
#from crazyflie_RL.rl_model import RlModel

def main():
    drone_id = 'cf20'
    final_target = np.array([2.5,2,0.2])

    #model_path = "data/SAC_save-12.07.2024_22.03.53/best_model.zip"
    #model = RlModel(model_path)

    rclpy.init()

    node = DronePosition(drone_id,final_target)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('test')
    
    """
    pos = np.zeros((1,3))
    vel = np.zeros((1,3))
    rpy = np.zeros((1,3))
    ang_v = np.zeros((1,3))

    obs = {
        "Position": np.array([pos[i,:] for i in range(1)]).astype('float32'),
        "Velocity": np.array([vel[i,:] for i in range(1)]).astype('float32'),
        "rpy": np.array([rpy[i,:] for i in range(1)]).astype('float32'),
        "ang_v": np.array([ang_v[i,:] for i in range(1)]).astype('float32'),
    }

    """

if __name__ == "__main__":
    main()

class RlModel():
    def __init__(self,model_path):

        self.model = SAC.load(Path(__file__).parent / model_path)

    def get_action(self,obs):
        action, _states = self.model.predict(obs,deterministic=True)   

        return action    
    
    def step(self,action_,pos_):
        pos = pos_[0]
        action = action_[0]
        target_pos = pos[0:3]+0.1*np.array([action[0],action[1],0])
        return target_pos

class DronePosition(Node):
    def __init__(self,drone_id,final_target):
        super().__init__('drone_position_node')
        self.drone_id = drone_id
        self.final_target = final_target

        self.publisher = self.create_publisher(String, 'drone_command', 10)

        self.command = 'idle'

        # Create a buffer and listener for TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1/240 , self.run_drone)
        self.prev_time = None

        self.prev_pos = None
        self.prev_rpy = None
        self.pos = np.zeros((1,3))
        self.vel = np.zeros((1,3))
        self.rpy = np.zeros((1,3))
        self.ang_v = np.zeros((1,3))
        
        self.obs = None
        self.log_obs = Logger_obs()

        self.min_pos = np.array([0,0,0])
        self.max_pos = np.array([5,4,1]) # if the drone goes outside of these boundrys it stops

        self.translation_world_to_local = [-2.0, 2.5, 0.0]
        rotation_world_to_local_eul = [0.0, 0.0, -np.pi / 2]
        self.rotation_world_to_local_qua = tf_transformations.quaternion_from_euler(rotation_world_to_local_eul[0], rotation_world_to_local_eul[1], rotation_world_to_local_eul[2])

        model_path = "data/SAC_save-12.07.2024_22.03.53/best_model.zip"
        self.model = RlModel(model_path)

    def run_drone(self):
        try:
            self.obs = self.get_obs()
            self.log_obs.log_obs(self.obs)
            transform = self.tf_buffer.lookup_transform('world', 'local_frame', rclpy.time.Time())
            self.alive = self.alive_check()

            if self.alive:
                if np.linalg.norm(self.final_target[0:2]-self.pos[0,0:2]) < 0.2:
                    self.command = 'land'
                    self.sendCommand()
                    self.log_obs.save_obs()
                    self.destroy_timer(self.timer)
                    
                else:
                    action = self.model.get_action(self.obs)
                    print(action)
                    local_target = self.model.step(action,self.pos)
                    world_target = self.local_to_world(local_target,transform)
                    self.getCommandgoTo(world_target)

            self.sendCommand()

            self.get_logger().info(f"Position: x={self.pos[0,0]}, y={self.pos[0,1]}, z={self.pos[0,2]}")
            #self.get_logger().info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")

    def get_obs(self):
        transform: TransformStamped = self.tf_buffer.lookup_transform('local_frame', self.drone_id, rclpy.time.Time())
            
        position = transform.transform.translation
        quaternion = transform.transform.rotation
        current_time = self.get_clock().now()

        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )

        self.pos = np.array([[position.x,position.y,position.z]])
        self.rpy = np.array([[roll,pitch,yaw]])

        if self.prev_pos is not None and self.prev_time is not None:
            # Calculate time difference (seconds)
            time_diff = (current_time - self.prev_time).nanoseconds / 1e9

            self.vel = (self.pos - self.prev_pos)/time_diff

            if self.prev_rpy is not None:
                self.ang_v = (self.rpy - self.prev_rpy)/time_diff

        self.prev_pos = self.pos
        self.prev_rpy = self.rpy
        self.prev_time = current_time

        obs = {
            "Position": np.array([self.pos[i,:] for i in range(1)]).astype('float32'),
            "Velocity": np.array([self.vel[i,:] for i in range(1)]).astype('float32'),
            "rpy": np.array([self.rpy[i,:] for i in range(1)]).astype('float32'),
            "ang_v": np.array([self.ang_v[i,:] for i in range(1)]).astype('float32'),
        }
        
        return obs

    def alive_check(self):
        if (self.min_pos[0] < self.pos[0,0] < self.max_pos[0] and
            self.min_pos[1] < self.pos[0,1] < self.max_pos[1] and
            self.min_pos[2] < self.pos[0,2] < self.max_pos[2]):
            alive = True
        else:
            alive = False

        if not alive:
            self.command = 'emergency'
            self.sendCommand()
            self.log_obs.save_obs()
            self.destroy_timer(self.timer)

        return alive
    
    def sendCommand(self):
        command_msg = String()
        command_msg.data = self.command
        self.publisher.publish(command_msg)
        self.get_logger().info(f"Published command: {command_msg.data}")

    def getCommandgoTo(self,target):
        self.command = f"goTo {target[0]} {target[1]}"

    def local_to_world(self,local_coordinates,transform):
        
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Create transformation matrix from rotation (quaternion) and translation
        translation_vector = np.array([translation.x, translation.y, translation.z])
        rotation_quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        transformation_matrix = tf_transformations.quaternion_matrix(rotation_quaternion)
        transformation_matrix[:3, 3] = translation_vector

        # Convert local coordinates to homogeneous coordinates
        local_homogeneous = np.array([local_coordinates[0], local_coordinates[1], local_coordinates[2], 1.0])

        # Apply transformation matrix to get world coordinates
        world_homogeneous = np.dot(transformation_matrix, local_homogeneous)

        return world_homogeneous

class Logger_obs():
    def __init__(self):

        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.output_folder= Path(__file__).parent / 'results'
        self.file_name = str(self.output_folder / f"obs_log_{current_time}.txt")

        self.all_obs = []

        print('logpath created')

    def log_obs(self,obs):
        flattened_obs = [
            obs["Position"][0][0], obs["Position"][0][1], obs["Position"][0][2],  # Position (x, y, z)
            obs["Velocity"][0][0], obs["Velocity"][0][1], obs["Velocity"][0][2],  # Velocity (x, y, z)
            obs["rpy"][0][0], obs["rpy"][0][1], obs["rpy"][0][2],                # Roll, Pitch, Yaw
            obs["ang_v"][0][0], obs["ang_v"][0][1], obs["ang_v"][0][2]          # Angular velocity (x, y, z)
        ]
        self.all_obs.append(flattened_obs)  # Append the observation to the log

    def save_obs(self):
        headers = [
            "Position_x", "Position_y", "Position_z",
            "Velocity_x", "Velocity_y", "Velocity_z",
            "Roll", "Pitch", "Yaw",
            "AngularVelocity_x", "AngularVelocity_y", "AngularVelocity_z"
        ]
        
        # Write the collected data to a CSV file
        with open(self.file_name, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(headers)  # Write the header row
            writer.writerows(self.all_obs)  # Write all the collected data
            print("observation written to file "+ self.file_name)
