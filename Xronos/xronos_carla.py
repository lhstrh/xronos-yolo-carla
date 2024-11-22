import xronos

import carla
import torch
import cv2
from carla_agent import game_start, game_step, game_stop, agent_start, agent_action, write_video, hydra_dir, agents_dict, cfg
from time_logger import TimeLogger
from tqdm import tqdm
from time import time
from datetime import timedelta
from pathlib import Path
import statistics
import numpy as np

class Carla(xronos.Reactor):
    env = None
    list_render = None
    timelogger = None
    actions = xronos.InputPortDeclaration()
    observation = xronos.OutputPortDeclaration()

    @xronos.reaction
    def on_startup(self, interface):
        interface.add_trigger(self.startup)
        observation = interface.add_effect(self.observation)
        def handler():
            self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="xronos_carla", save_dir=hydra_dir + "/time")
            self.env, init_obs = game_start()
            self.timelogger.mark_start()
            self.list_render = []
            print("===== Carla Startup =====")
            self.timelogger.mark_end()
            observation.value = init_obs
            self.timelogger.start()
            self.inference_total = 0
        return handler

    @xronos.reaction
    def on_actions(self, interface):
        actions = interface.add_trigger(self.actions)
        observation = interface.add_effect(self.observation)
        def handler():
            self.timelogger.mark_start()
            if(actions.value == None):
                raise Exception("Actions is None")
            control_dict = {}
            self.timelogger.log()
            for actor_id, agent in agents_dict.items():
                control_dict[actor_id] = actions.value
            obs, reward, done, info, tiled_img, single_img = game_step(self.env, actions.value)
            
            if cfg.log_video:
                self.list_render.append(tiled_img)

            if self.env.timestamp["step"] > cfg.total_steps:
                if cfg.log_video:
                    write_video(self.list_render)
                game_stop(self.env)
                return
            self.timelogger.mark_end()
            observation.value = obs
        return handler

class AI_Agent(xronos.Reactor):
    timelogger = None
    obs = xronos.InputPortDeclaration()
    ai_actions = xronos.OutputPortDeclaration()

    @xronos.reaction
    def on_startup(self, interface):
        interface.add_trigger(self.startup)
        def hander():
            self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="xronos_agent", save_dir=hydra_dir + "/time")
            print("===== Agent Startup =====")
            agent_start()
        return hander
    
    @xronos.reaction
    def on_obs(self, interface):
        obs = interface.add_trigger(self.obs)
        ai_actions = interface.add_effect(self.ai_actions)
        def handler():
            self.timelogger.mark_start()
            if(obs.value == None):
                raise Exception("Obs is None")
            actions = agent_action(obs.value)
            self.timelogger.mark_end()
            ai_actions.value = actions
        return handler
    
class Yolo(xronos.Reactor):
    yolo_results = None
    yolo_model = None
    count = None
    t0 = None
    t_last = None
    out = None
    timelogger = None
    img = xronos.InputPortDeclaration()
    yolo_actions = xronos.OutputPortDeclaration()

    @xronos.reaction
    def on_startup(self, interface):
        interface.add_trigger(self.startup)
        def hander():
            print("===== YOLO Startup =====")
            self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="xronos_yolo", save_dir=hydra_dir + "/time")
            self.yolo_model = torch.hub.load("ultralytics/yolov5", "yolov5n", pretrained=True)
            self.yolo_results = []
            self.count = 0
        return hander
    
    @xronos.reaction
    def on_img(self, interface):
        img = interface.add_trigger(self.img)
        yolo_actions = interface.add_effect(self.yolo_actions)
        def handler():
            self.timelogger.mark_start()
            result = self.yolo_model(img.value["hero"]["central_rgb"]["data"])
            if cfg.log_video:
                self.yolo_results.append(result)
                frame = result.render()[0]
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if(self.count == 0):
                    self.t0 = time()
                    self.t_last = time()
                    height, width, _ = frame_bgr.shape
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    yolo_path = hydra_dir + "/videos/xronos_yolo.mp4"
                    print("yolo_path:", yolo_path)
                    self.out = cv2.VideoWriter(yolo_path, fourcc, 25.0, (width, height))
                t_now = time()
                t_elapsed = t_now - self.t_last
                self.t_last = t_now
                seconds = int(t_now - self.t0)
                milliseconds = int((t_now - self.t0) * 1000)
                cv2.putText(frame_bgr, f"Xronos", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame_bgr, f"Wall clock {seconds}s, {0.0 if self.count<1 else 1/t_elapsed:.1f} FPS", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                output = cv2.putText(frame_bgr, f"Total Frames {self.count}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                self.out.write(output)
                if self.count >=cfg.total_steps-1:
                    self.out.release()
                    cv2.destroyAllWindows()
                    print("==== YOLO LOGGED ====")
            self.count += 1
            self.timelogger.mark_end()
            yolo_actions.value = None
        return handler

class Fusion(xronos.Reactor):
    timelogger = None
    ai_actions = xronos.InputPortDeclaration()
    yolo_actions = xronos.InputPortDeclaration()
    final_actions = xronos.OutputPortDeclaration()

    @xronos.reaction
    def on_startup(self, interface):
        interface.add_trigger(self.startup)
        def hander():
            print("===== Fusion Startup =====")
            self.timelogger = TimeLogger(warmup_steps = cfg.warmup_steps, total_steps = cfg.total_steps, name="xronos_fusion", save_dir=hydra_dir + "/time")
        return hander
    
    @xronos.reaction
    def on_obs(self, interface):
        yolo_actions = interface.add_trigger(self.yolo_actions)
        ai_actions = interface.add_trigger(self.ai_actions)
        final_actions = interface.add_effect(self.final_actions)
        def handler():
            self.timelogger.mark_start()
            self.timelogger.mark_end()
            final_actions.value = ai_actions.value
        return handler


env = xronos.Environment()
carla = env.create_reactor("carla", Carla)
ai_agent = env.create_reactor("ai_agent", AI_Agent)
yolo = env.create_reactor("yolo", Yolo)
fusion = env.create_reactor("fusion", Fusion)

env.connect(carla.observation, ai_agent.obs)
env.connect(ai_agent.ai_actions, fusion.ai_actions)

env.connect(carla.observation, yolo.img)
env.connect(yolo.yolo_actions, fusion.yolo_actions)

env.connect(fusion.final_actions, carla.actions, delay=timedelta(0))

env.execute()