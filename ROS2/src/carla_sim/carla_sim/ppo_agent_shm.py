from message_def.srv import CarlaObs
from message_def.msg import BytesMsg, ControlMsg
from multiprocessing import shared_memory
from message_passing_shm import Serializer

import rclpy
from rclpy.node import Node
import pickle
from time_logger import TimeLogger

import torch
import cv2
from carla_agent import game_start, game_step, game_stop, agent_start, agent_action, write_video, hydra_dir, agents_dict, cfg
from tqdm import tqdm
from time import time
from pathlib import Path
import carla

class PPOAgent(Node):
    def __init__(self):
        super().__init__('ppo_agent')
        print("======== PPO Agent Pubsub Shared Memory Start ========")
        self.publisher_ = self.create_publisher(
            ControlMsg, 'ppo_action_topic', 10)
        self.subscription = self.create_subscription(
            BytesMsg,
            'agent_obs_topic',
            self.listener_callback,
            10)
        self.seri = Serializer()
        self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="ros2_shm_agent", save_dir=hydra_dir + "/time")
        agent_start()
        print("======== PPO Agent Pubsub Shared Memory Ready ========")
    
    def listener_callback(self, msg):
        agent_obs = self.seri.deserialize(pickle.loads(b''.join(msg.data)))
        self.timelogger.mark_start()
        obs = agent_obs

        control_dict = agent_action(obs)
        for actor_id, agent in agents_dict.items():
            control_dict[actor_id] = {
            "throttle": control_dict[actor_id].throttle,
            "steer": control_dict[actor_id].steer,
            "brake": control_dict[actor_id].brake,
        }
        print("Action:", control_dict)
        self.timelogger.mark_end()
        self.send_message(control_dict)

    def send_message(self, val):
        msg = ControlMsg()
        msg.throttle = val['hero']['throttle']
        msg.steer = val['hero']['steer']
        msg.brake = val['hero']['brake']
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    policy_node = PPOAgent()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()