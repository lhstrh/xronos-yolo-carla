import requests
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--server_controller_url', type=str, help='The URL of the server controller', required=True)
args = parser.parse_args()
carl_server_controller_url = args.server_controller_url

def check_server_controller_health():
    try:
        response = requests.get(carl_server_controller_url, timeout=1)
    except:
        raise Exception("carla_server_controller inactive.")
    if response.text!="Healthy":
        raise Exception("carla_server_controller unhealthy.")
        
check_server_controller_health()
print("Carla server controller healthy and running!")