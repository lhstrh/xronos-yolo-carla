import yaml

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--carla_ip', type=str, help='The IP address of CARLA server', required=True)
parser.add_argument('--server_controller_url', type=str, help='The URL of the server controller', required=True)
parser.add_argument('--config_file', type=str, help='The path to the config file you want to modify', required=True)
args = parser.parse_args()

with open(args.config_file) as f:
    config = yaml.safe_load(f)

config['host'] = args.carla_ip
config['carla_controller_url'] = args.server_controller_url

with open(args.config_file, "w") as f:
    yaml.safe_dump(config, f, sort_keys=False)