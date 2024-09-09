import numpy as np
import pickle

def check(value):
    from collections.abc import Sequence
    from collections.abc import Set
    from collections import UserList
    from collections import UserString
    assert \
        ((isinstance(value, Sequence) or
            isinstance(value, Set) or
            isinstance(value, UserList)) and
            not isinstance(value, str) and
            not isinstance(value, UserString) and
            all(isinstance(v, bytes) for v in value) and
            True), \
        "The 'obs' field must be a set or sequence and each value of type 'bytes'"

np_array = np.array([1, 2, 3, 4, 5])

serialized_bytes = pickle.dumps(np_array)
serialized_bytes = bytes([bytes([b]) for b in serialized_bytes])
check(serialized_bytes)

print(type(serialized_bytes))
deserialized_np_array = pickle.loads(serialized_bytes)
deserialization_successful = np.array_equal(np_array, deserialized_np_array)

print("deserialization_successful", deserialization_successful)