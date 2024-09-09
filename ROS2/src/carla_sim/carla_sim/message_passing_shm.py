import time
import numpy as np
import pickle
import numpy as np
from time import time
from multiprocessing import shared_memory

class Serialized_NumPy():
    def __init__(self, object_id, size):
        self.object_id = object_id
        self.size = size
    def read(self):
        return self.object_id, self.size

class Serializer():
    def serialize(self, data):
        serialized_array = pickle.dumps(data)
        ob_size = len(serialized_array)
        # print('serialized_array len:', ob_size)
        # Create and Write shared memory
        local_shm = shared_memory.SharedMemory(create=True, size=ob_size)
        local_shm.buf[:ob_size] = serialized_array
        return (local_shm.name, ob_size)
    def deserialize(self, obj):
        ob_name, ob_size = obj
        local_shm = shared_memory.SharedMemory(name=ob_name)
        data = pickle.loads(local_shm.buf[:ob_size])
        # local_shm.close()
        # local_shm.unlink()
        return data

def deep_compare(obj1, obj2):
    """Recursively compare two objects."""
    if type(obj1) != type(obj2):
        return False
    if isinstance(obj1, np.ndarray):
        return np.array_equal(obj1, obj2)
    if isinstance(obj1, dict):
        if obj1.keys() != obj2.keys():
            return False
        return all(deep_compare(obj1[key], obj2[key]) for key in obj1)
    elif isinstance(obj1, (list, tuple)):
        if len(obj1) != len(obj2):
            return False
        return all(deep_compare(item1, item2) for item1, item2 in zip(obj1, obj2))
    else:
        return obj1 == obj2
    
def test_serializer(serializer, input, do_print = False):
    serialized_data = serializer.serialize(input)
    output = serializer.deserialize(serialized_data)
    # if not deep_compare(input, output):
    #     if do_print:
    #         print("!!! Error !!!")
    #         print("Input:", input)
    #         print("Output:", output)
    #     raise Exception("Input Does't Match Output")
    # else: 
    #     if do_print:
    #         print("== Success ==")

if __name__ =='__main__':
    print("======== Test Message Passing ========")
    test_times = 100
    t0 = time()
    serializer = Serializer()
    for i in range(test_times):
        test_serializer(serializer, {"entry": "entry1", "another entry": "entry 2"})
        test_serializer(serializer, np.ones(1310720))
        test_serializer(serializer, {"entry": np.ones(1310720), "another entry": "entry 2"})
        # Test Yolo
        with open("./single_img.pkl", 'rb') as file:
            input = pickle.load(file)
        test_serializer(serializer, input)
    t1 = time()
    print("Time Per Cycle:", (t1-t0) / test_times)
    print("======== Message Passing Testsed ========")