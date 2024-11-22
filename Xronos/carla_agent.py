import sys
import os
sys.path.insert(0, os.environ['MILE_ROOT'])
os.chdir(os.environ['MILE_ROOT'])
# print("CWD:", os.getcwd())
# print("PATH:", sys.path)
from hydra import compose, initialize
from omegaconf import DictConfig, OmegaConf
import logging
from myutils import server_utils_controller
from carla_gym.utils import config_utils
from pathlib import Path
import logging

import numpy as np
import gym
from stable_baselines3.common.vec_env.base_vec_env import tile_images
from gym.wrappers.monitoring.video_recorder import ImageEncoder
from datetime import datetime

log = logging.getLogger(__name__)
CARLA_FPS = 25

# initialize(config_path="../../config", version_base="1.1")
initialize(config_path="./config", version_base="1.1")
cfg = compose("carla.yaml")

if cfg.force_ppo_cpu:
    os.environ["FORCE_PPO_CPU"] = "TRUE"
else:
    os.environ["FORCE_PPO_CPU"] = "FALSE"
hydra_dir = os.getcwd() + "/logs"
print("hydra_dir:", hydra_dir)
os.makedirs(hydra_dir,exist_ok=True)
os.chdir(hydra_dir)
print("LOG_VIDEOS", cfg.log_video)
log.setLevel(getattr(logging, cfg.log_level.upper()))
server_manager = server_utils_controller.CarlaServerManager(cfg.carla_sh_path, port=cfg.port, controller_url=cfg.carla_controller_url)
server_manager.kill_carla()

data_writer = None
agents_dict = {}

def agent_start():
    print("======== Agent Init ========")
    agent_names = []
    # single actor, place holder for multi actors    
    for ev_id, ev_cfg in cfg.actors.items():
        agent_names.append(ev_cfg.agent)
        cfg_agent = cfg.agent[ev_cfg.agent]
        OmegaConf.save(config=cfg_agent, f='config_agent.yaml')
        AgentClass = config_utils.load_entry_point(cfg_agent.entry_point)
        agents_dict[ev_id] = AgentClass('config_agent.yaml')

    print("======== Agent Inited ========")

def game_start():
    print("======== Game Start ========")
    # Start Carla Server
    server_manager.start()

    obs_configs = {}
    reward_configs = {}
    terminal_configs = {}
    agent_names = []
    # single actor, place holder for multi actors    
    for ev_id, ev_cfg in cfg.actors.items():
        agent_names.append(ev_cfg.agent)
        cfg_agent = cfg.agent[ev_cfg.agent]
        OmegaConf.save(config=cfg_agent, f='config_agent.yaml')
        AgentClass = config_utils.load_entry_point(cfg_agent.entry_point)
        agents_dict[ev_id] = AgentClass('config_agent.yaml')
        obs_configs[ev_id] = agents_dict[ev_id].obs_configs

        # get obs_configs from agent
        reward_configs[ev_id] = OmegaConf.to_container(ev_cfg.reward)
        terminal_configs[ev_id] = OmegaConf.to_container(ev_cfg.terminal)

    # check h5 birdview maps have been generated
    config_utils.check_h5_maps(cfg.test_suites, obs_configs, cfg.carla_sh_path)

    # compose suite_name
    env_idx = 0
    env_setup = OmegaConf.to_container(cfg.test_suites[env_idx])
    suite_name = '-'.join(agent_names) + '_' + env_setup['env_id']
    for k in sorted(env_setup['env_configs']):
        suite_name = suite_name + '_' + str(env_setup['env_configs'][k])

    log.info(f"Start Benchmarking! env_idx: {env_idx}, suite_name: {suite_name}")

    # make directories
    diags_dir = Path('diagnostics')
    driver_log_dir = Path('driver_log')
    video_dir = Path('videos')
    diags_dir.mkdir(parents=True, exist_ok=True)
    driver_log_dir.mkdir(parents=True, exist_ok=True)
    video_dir.mkdir(parents=True, exist_ok=True)

    print("======== Making ENV ========")
    print("obs_configs:", obs_configs)
    # make env
    env_setup = OmegaConf.to_container(cfg.test_suites[env_idx])
    env = gym.make(env_setup['env_id'], obs_configs=obs_configs, reward_configs=reward_configs,
                   terminal_configs=terminal_configs, host=cfg.host, port=cfg.port,
                   seed=cfg.seed, no_rendering=cfg.no_rendering, **env_setup['env_configs'])
    
    env.set_task_idx(np.random.choice(env.num_tasks))
    run_name = f"{env.task['weather']}_{env.task['route_id']:02d}"
    print("======== Return ENV ========")
    init_obs = env.reset()
    return env, init_obs

def agent_action(obs):
    control_dict = {}
    for actor_id, agent in agents_dict.items():
        control_dict[actor_id] = agent.run_step(obs[actor_id])
    return control_dict

def game_step(env, control_dict):
    # Step Environment
    obs, reward, done, info = env.step(control_dict)
    # Log Image
    render_imgs = []
    single_img = None
    tiled_img = None
    for actor_id, agent in agents_dict.items():
        single_img = obs[actor_id]['central_rgb']['data']
        if cfg.log_video:
            render_imgs.append(single_img)
        if done[actor_id]:
            print("DONE:", info[actor_id]['episode_stat'], info[actor_id]['episode_event'])
    if cfg.log_video:
        tiled_img = tile_images(render_imgs)
    if cfg.agent == 'ppo':
        obs[actor_id].pop('central_rgb')
    return obs, reward, done, info, tiled_img, single_img

def game_stop(env):
    env.close()
    env = None
    server_manager.stop()
    print("======== Game End ========")

def write_video(list_render):
    print("==== LOGGING VIDEO ====")
    video_path = hydra_dir + "/videos/run.mp4"
    print("video_path:", video_path)
    encoder = ImageEncoder(video_path, list_render[0].shape, 2*CARLA_FPS, 2*CARLA_FPS)
    for im in list_render:
        encoder.capture_frame(im)
    encoder.close()
    print("==== VIDEO LOGGED ====")
    encoder = None
