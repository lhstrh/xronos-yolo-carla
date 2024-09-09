from message_def.srv import CarlaObs, YoloObs
from message_def.msg import BytesMsg, ControlMsg

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


class Fusion(Node):

    def __init__(self):
        super().__init__('fusion')
        print("======== Fusion Pubsub Start ========")
        # Subscribers
        # self.ppo_agent = Subscriber(self, BytesMsg, 'ppo_action_topic')
        # self.yolo_agent = Subscriber(self, BytesMsg, 'yolo_action_topic')
        # # Synchronize the subscribers
        # ats = TimeSynchronizer(
        #     [self.ppo_agent, self.yolo_agent], 1000)
        # ats.registerCallback(self.atscallback)
        topics = ['ppo_action_topic', 'yolo_action_topic']
        self.states = {topic: None for topic in topics}
        self.updated = {topic: False for topic in topics}
        for topic in topics:
            self.create_subscription(
                ControlMsg, topic, lambda msg, topic=topic: self.policy_callback(msg, topic), 10)
        # Publisher
        self.publisher_ = self.create_publisher(
            ControlMsg, 'fusion_action_topic', 10)
        self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="ros2_shm_fusion", save_dir=hydra_dir + "/time")
        print("======== Fusion Pubsub Ready ========")

    def policy_callback(self, msg, topic_name):
        self.states[topic_name] = msg
        self.updated[topic_name] = True
        if all(self.updated.values()):
            self.timelogger.mark_start()
            print("Sending message:", self.states['ppo_action_topic'])
            self.send_message(self.states['ppo_action_topic'])
            self.timelogger.mark_end()
            self.reset_for_next_round()
    
    def reset_for_next_round(self):
        # Reset states and updated flags for the next round
        for key in self.states.keys():
            self.states[key] = None
            self.updated[key] = False

    def atscallback(self, ppo_input, yolo_input):
        ppo_action = pickle.loads(ppo_input.data)
        yolo_action = pickle.loads(yolo_input.data)
        # print("yolo_action", yolo_action)
        control_dict = {}
        self.t2 = time()
        for actor_id, agent in agents_dict.items():
            control_dict[actor_id] = carla.VehicleControl(throttle=ppo_action[actor_id]["throttle"], steer=ppo_action[actor_id]["steer"], brake=ppo_action[actor_id]["brake"])
        self.step_env(control_dict)

    def send_message(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = Fusion()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()