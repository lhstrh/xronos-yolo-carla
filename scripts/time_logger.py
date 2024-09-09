
import time
from tqdm import tqdm
import pickle
import os
class TimeLogger():
    def __init__(self, warmup_steps = 100, total_steps = 1000, name = "default", save_dir = "./logs"):
        self.t0 = 0
        self.t1 = 0
        self.last_frame = 0
        self.frame_times = []
        self.marks = []
        self.name = name
        self.frame_count = 0
        self.warmup_steps = warmup_steps
        self.total_steps = total_steps - 2
        self.save_dir = save_dir
        self.t_warmup = None
        self.pbar = tqdm(total=total_steps)
        self.start()
    
    def start(self):
        self.t0 = time.time()
        self.last_frame = self.t0

    def log(self):
        t_cur = time.time()
        self.frame_times.append(t_cur - self.t0)
        self.last_frame = t_cur
        self.t1 = t_cur
        self.frame_count += 1

        if(self.frame_count == self.warmup_steps):
            self.t_warmup = t_cur
        self.pbar.set_description("FPS: {:.2f}".format(self.fps()), refresh=True)
        self.pbar.update(1)

        if self.frame_count == self.total_steps:
            self.save_logs()

    def mark_start(self):
        self.last_start = time.time()

    def mark_end(self):
        t = time.time()
        mark = {
            "start": self.last_start,
            "duration": t - self.last_start,
            "end": t,
            "name": self.name
        }
        self.marks.append(mark)
        if len(self.marks) == self.total_steps:
            self.save_marks()

    def save_marks(self):
        path = self.save_dir + "/" + self.name + "_marks.pkl"
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            pickle.dump(self.marks, f)
    
    def fps(self):
        if(self.t_warmup and self.frame_count > self.warmup_steps):
            return (self.frame_count - self.warmup_steps) / (self.t1 - self.t_warmup)
        return self.frame_count / (self.t1 - self.t0)

    def save_logs(self):
        self.pbar.close()
        path = self.save_dir + "/" + self.name + "_logs.pkl"
        print(f"Final FPS is {self.fps()} in {self.frame_count} frames. \n")
        print(f"Saving Logs to {path}")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            pickle.dump(self.frame_times, f)