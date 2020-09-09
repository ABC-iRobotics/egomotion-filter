from numba import cuda
import numpy as np



@cuda.jit
def max_example_3d(result, values):
    """
    Find the maximum value in values and store in result[0].
    Both result and values are 3d arrays.
    """
    i, j, k = cuda.grid(3)
    # Atomically store to result[0,1,2] from values[i, j, k]
    cuda.atomic.max(result, (0, 1, 2), values[i, j, k])

arr = np.random.rand(1000).reshape(10,10,10)
result = np.zeros((3, 3, 3), dtype=np.float64)
max_example_3d[(2, 2, 2), (5, 5, 5)](result, arr)
print(result[0, 1, 2], '==', np.max(arr))
