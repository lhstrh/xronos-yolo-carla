import torch

cuda_available = torch.cuda.is_available()

print("CUDA Available:", cuda_available)
torch.hub.load("ultralytics/yolov5", "yolov5n", pretrained=True)

if cuda_available:
    num_cuda_devices = torch.cuda.device_count()
    print("Number of CUDA Devices:", num_cuda_devices)
    for i in range(num_cuda_devices):
        print(f"CUDA Device {i}: {torch.cuda.get_device_name(i)}")
else:
    print("No CUDA devices are available.")
    # raise Exception("No CUDA device available")
