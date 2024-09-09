"""Adapted from https://github.com/zhejz/carla-roach CC-BY-NC 4.0 license."""

import os
import time
from omegaconf import OmegaConf
import logging
import requests

log = logging.getLogger(__name__)

from mile.constants import CARLA_FPS

class CarlaServerManager():
    def __init__(self, carla_sh_str, port=2000, controller_url = "", configs=None, t_sleep=5):
        self._carla_sh_str = carla_sh_str
        self.port = port
        if(controller_url[-1]!='/'):
            controller_url = controller_url + "/"
        self.carl_server_controller_url = controller_url
        print("Carla Controller URL:", self.carl_server_controller_url)
        # self._root_save_dir = root_save_dir
        self._t_sleep = t_sleep
        self.env_configs = []

        if configs is None:
            cfg = {
                'gpu': os.environ.get('CUDA_VISIBLE_DEVICES'),
                'port': port,
            }
            self.env_configs.append(cfg)
        else:
            for cfg in configs:
                for gpu in cfg['gpu']:
                    single_env_cfg = OmegaConf.to_container(cfg)
                    single_env_cfg['gpu'] = gpu
                    single_env_cfg['port'] = port
                    self.env_configs.append(single_env_cfg)
                    port += 5
    def kill_carla(self):
        try:
            response = requests.post(self.carl_server_controller_url+'kill_carla')
        except Exception as e:
            raise Exception("Error killing CARLA:", e)
        if response.text!='success':
            raise Exception("Error killing CARLA:", response.text)    
        log.info(f"Killed Carla Servers with remote controller on {self.port}!")
        time.sleep(1)

    def start(self):
        for cfg in self.env_configs:
            cmd = f'-fps={CARLA_FPS} -quality-level=Epic -carla-rpc-port={cfg["port"]}'
            log.info(cmd)
            try:
                response = requests.post(self.carl_server_controller_url+'start_carla', data=cmd)
            except Exception as e:
                raise Exception("Error starting CARLA:", e)
            if response.text!='success':
                raise Exception("Error starting CARLA:", response.text)
        time.sleep(self._t_sleep)

    def stop(self):
        self.kill_carla()
        time.sleep(self._t_sleep)
