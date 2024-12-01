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
]
labels = [
    "ROS2",
    "ROS2 Shared Memory",
    "HPRM Decentralized",
    "HPRM Centralized",
]
plt.figure(figsize=(8, 5), dpi=300)
plt.boxplot(data, labels=labels)
plt.ylabel('Inference Latency (ms)')
plt.yscale('log')
y_ticks = [10, 20, 40, 80, 160, 320]
plt.yticks(y_ticks, [str(tick) for tick in y_ticks])

means = [np.mean(dat) for dat in data]

plt.savefig("./logs/latency_plot.png", bbox_inches='tight') 
print(means)