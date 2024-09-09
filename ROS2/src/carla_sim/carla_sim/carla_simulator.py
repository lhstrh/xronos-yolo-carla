import sys
import rclpy
from rclpy.node import Node
from message_def.srv import AddThreeInts, CarlaObs, YoloObs
from message_filters import TimeSynchronizer, Subscriber
from message_def.msg import BytesMsg, ControlMsg

import torch
import cv2
from carla_agent import game_start, game_step, game_stop, agent_start, agent_action, write_video, hydra_dir, agents_dict, cfg
from tqdm import tqdm
from time import time
from pathlib import Path
from time_logger import TimeLogger
import carla
import pickle
import statistics
import numpy as np

class CarlaEnv(Node):
    def __init__(self):
        super().__init__('env')
        print("======== Carla Pubsub Start ========")
        # Subscribers
        self.subscription = self.create_subscription(
            ControlMsg,
            'fusion_action_topic',
            self.listener_callback,
            10)

        # Publisher
        self.agent_obs_publisher = self.create_publisher(
            BytesMsg, 'agent_obs_topic', 10)
        self.yolo_obs_publisher = self.create_publisher(
            BytesMsg, 'yolo_obs_topic', 10)
        
        self.env, init_obs = game_start()
        self.list_render = []
        self.t1 = time()
        self.inference_total = 0
        self.fps_log = []
        print("======== Carla Pubsub Ready ========")
        self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="ros2_s_carla", save_dir=hydra_dir + "/time")
        self.timelogger.mark_start()
        self.agent_finish = False
        self.yolo_finish = False
        self.timelogger.mark_end()
        self.send_agent_obs(init_obs)
        self.send_yolo_obs(init_obs['hero']['central_rgb']['data'])

    def step_env(self, action):
        control_dict = {}
        self.timelogger.log()
        for actor_id, agent in agents_dict.items():
            control_dict[actor_id] = carla.VehicleControl(throttle=action[actor_id]["throttle"], steer=action[actor_id]["steer"], brake=action[actor_id]["brake"])
        obs, reward, done, info, tiled_img, single_img = game_step(self.env, control_dict)
        
        if cfg.log_video:
            self.list_render.append(tiled_img)
       
        if self.env.timestamp["step"] > cfg.total_steps:
            if cfg.log_video:
                write_video(self.list_render)
            game_stop(self.env)
            return
        self.timelogger.mark_end()
        self.send_agent_obs(obs)
        self.send_yolo_obs(single_img)  
    
    def listener_callback(self, msg):
        data = {
            'hero': {
                'throttle': msg.throttle,
                'steer': msg.steer,
                'brake': msg.brake,
            }
        }
        self.timelogger.mark_start()
        self.step_env(data)

    def send_agent_obs(self, obs):
        msg = BytesMsg()
        msg.data = pickle.dumps(obs)
        self.agent_obs_publisher.publish(msg)
        
    def send_yolo_obs(self, img):
        msg = BytesMsg()
        msg.data = pickle.dumps(img)
        self.yolo_obs_publisher.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    env_node = CarlaEnv()
    rclpy.spin(env_node)
    env_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()