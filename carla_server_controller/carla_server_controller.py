import psutil
import subprocess
import argparse
import os
from flask import Flask, request

app = Flask(__name__)
parser = argparse.ArgumentParser()
parser.add_argument('--carlapath', type=str, help='Path to the Carla simulator.', required=True)
args = parser.parse_args()
CARLA_SERVER_PATH=args.carlapath
CARLA_SERVER_PORT=2000

print("CARLA_SERVER_PATH:", CARLA_SERVER_PATH)
if not os.path.exists(CARLA_SERVER_PATH):
    raise Exception("carlapath invalid")

def is_carla_running():
    for proc in psutil.process_iter(['pid', 'name']):
        try:
            connections = proc.connections()
            for conn in connections:
                if conn.laddr.port == CARLA_SERVER_PORT:
                    return True
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            pass
    return False

def kill_process_on_port(port):
    for proc in psutil.process_iter(['pid', 'name']):
        try:
            connections = proc.connections()
            for conn in connections:
                if conn.laddr.port == port:
                    print(f"Killing process {proc.pid} - {proc.name()} running on port {port}")
                    proc.kill()
                    return True
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            pass
    print(f"No process found running on port {port}")
    return False

def start_program(exe_path, params):
    print("Starting Carla Server with params: ", params)
    try:
        subprocess.Popen(exe_path + ' ' + params)
        print(f"Started program: {exe_path}")
    except FileNotFoundError:
        print(f"Error: File not found at {exe_path}")
    except Exception as e:
        print(f"Error: {e}")

@app.route('/')
def healthy():
    return "Healthy"

@app.route('/is_carla_running', methods=['GET'])
def is_carla_running_api():
    if is_carla_running():
        return "running"
    else:
        return "not running"

@app.route('/start_carla', methods=['POST'])
def start_carla():
    if is_carla_running():
        kill_process_on_port(CARLA_SERVER_PORT)

    params = request.data.decode('utf-8')
    print("START CARLA params:", params)
    start_program(CARLA_SERVER_PATH, params)
    return 'success'

@app.route('/kill_carla', methods=['POST'])
def kill_carla():
    if is_carla_running()==False:
        return 'success'
    kill_process_on_port(CARLA_SERVER_PORT)
    return 'success'

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=2010)
