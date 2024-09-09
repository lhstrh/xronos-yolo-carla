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


class YoloAgent(Node):

    def __init__(self):
        super().__init__('yolo_agent')
        print("======== Yolo Agent Pubsub Start ========")
        self.publisher_ = self.create_publisher(
            ControlMsg, 'yolo_action_topic', 10)
        self.subscription = self.create_subscription(
            BytesMsg,
            'yolo_obs_topic',
            self.listener_callback,
            10)

        self.yolo_model = torch.hub.load("ultralytics/yolov5", "yolov5n", pretrained=True)
        self.yolo_results = []
        self.count = 0
        self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="ros2_s_yolo", save_dir=hydra_dir + "/time")
        print("======== Yolo Agent Pubsub Ready ========")

    def listener_callback(self, msg):
        img = pickle.loads(b''.join(msg.data))
        self.timelogger.mark_start()
        print('Incoming request', img)
        result = self.yolo_model(img)
        if cfg.log_video:
            self.yolo_results.append(result)
            frame = result.render()[0]
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if(self.count == 0):
                self.t0 = time()
                self.t_last = time()
                height, width, _ = frame_bgr.shape
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                yolo_path = hydra_dir + "/videos/ros2_yolo.mp4"
                print("yolo_path:", yolo_path)
                self.out = cv2.VideoWriter(yolo_path, fourcc, 25.0, (width, height))
            t_now = time()
            t_elapsed = t_now - self.t_last
            self.t_last = t_now
            seconds = int(t_now - self.t0)
            milliseconds = int((t_now - self.t0) * 1000)
            cv2.putText(frame_bgr, f"ROS2", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame_bgr, f"Wall clock {seconds}s, {1/t_elapsed:.1f} FPS", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            output = cv2.putText(frame_bgr, f"Total Frames {self.count}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            self.out.write(output)
            if self.count >=cfg.total_steps-1:
                self.out.release()
                cv2.destroyAllWindows()
                print("==== YOLO LOGGED ====")
        self.count += 1
        print("send")
        self.timelogger.mark_end()
        self.send_message(123)

    def send_message(self, val):
        msg = ControlMsg()
        msg.throttle = 1.0
        msg.steer = 2.0
        msg.brake = 3.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    policy_node = YoloAgent()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()