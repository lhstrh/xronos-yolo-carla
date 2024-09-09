import pyarrow.plasma as plasma
import pickle
import pyarrow as pa
import pyarrow.plasma as plasma
import numpy as np

class Serialized_NumPy():
    def __init__(self, object_id):
        self.object_id = object_id
    def read(self):
        return self.object_id

class Serializer():
    def __init__(self, location="/tmp/plasma"):
        self.session = plasma.connect(location)
    def serialize_np(self, data):
        if not isinstance(data, np.ndarray):
            raise Exception("serialize_np: Trying to serialize non-np with plasma")
        tensor = pa.Tensor.from_numpy(np.ascontiguousarray(data))
        object_id = plasma.ObjectID(np.random.bytes(20))
        data_size = pa.ipc.get_tensor_size(tensor)
        buf = self.session.create(object_id, data_size)
        stream = pa.FixedSizeBufferWriter(buf)
        pa.ipc.write_tensor(tensor, stream)
        self.session.seal(object_id)
        return Serialized_NumPy(object_id)

    def deserialize_np(self, serialized_numpy):
        object_id = serialized_numpy.read()
        [buf2] = self.session.get_buffers([object_id])
        reader = pa.BufferReader(buf2)
        tensor2 = pa.ipc.read_tensor(reader)
        array = tensor2.to_numpy()
        return array
    def serialize_one(self, obj):
        if isinstance(obj, np.ndarray):
            return self.serialize_np(obj)
        if isinstance(obj, dict):
            for key in obj.keys():
                obj[key] = self.serialize_one(obj[key])
        return obj
    def deserialize_one(self, obj):
        if isinstance(obj, Serialized_NumPy):
            return self.deserialize_np(obj)
        if isinstance(obj, dict):
            for key in obj.keys():
                obj[key] = self.deserialize_one(obj[key])
        return obj
    def serialize(self, obj):
        return pickle.dumps(self.serialize_one(obj), protocol=5)
    def deserialize(self, obj):
        return self.deserialize_one(pickle.loads(obj))