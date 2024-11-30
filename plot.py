import pickle
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

directory = "./logs/time"

def analyze_inference(prefix: str):
    all_marks = {
        "agent": [],
        "carla": [],
        "yolo": [],
        "fusion": [],
    }
    for filepath in list(Path(directory).glob(prefix+"_*_marks.pkl")):
        rector_name = filepath.name.split("_")[2]
        if prefix == "xronos":
            rector_name = filepath.name.split("_")[1]
        with open(filepath, 'rb') as file:
            marks = pickle.load(file)
            all_marks[rector_name] = marks
    inference_times = []
    for i in range(100, len(all_marks["carla"]) - 1):
        lentency = all_marks["carla"][i+1]["start"] - all_marks["carla"][i]["end"]
        inference_times.append(lentency * 1000) # ms
    return inference_times

data = [
    analyze_inference("ros2_s"),
    analyze_inference("ros2_shm"),
    analyze_inference("lf_decentralized"),
    analyze_inference("lf_centralized"),
    analyze_inference("xronos"),
]
labels = [
    "ROS2",
    "ROS2 Shared Memory",
    "HPRM Decentralized",
    "HPRM Centralized",
    "Xronos"
]
plt.rcParams.update({'font.size': 12})
plt.figure(figsize=(12, 6), dpi=300)
plt.boxplot(data, labels=labels)
plt.ylabel('Inference Latency (ms)')
plt.savefig("./logs/latency_plot.png", bbox_inches='tight') 
print([f'{np.mean(dat):.2f}' for dat in data])