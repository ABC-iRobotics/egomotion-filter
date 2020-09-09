import time
import numpy as np
from numba import cuda
from numba import jit

width = 1024
height = 1024
size = width * height
print("Number of records")
print(size)

array_of_random = np.random.rand(size)
output_array = np.zeros(size, dtype=bool)
device_array = cuda.to_device(array_of_random)
device_output_array = cuda.device_array_like(output_array)



@cuda.jit()
def count_array_cuda(local_device_array, pivot_point, local_device_output_array):
    tx = cuda.threadIdx.x
    ty = cuda.blockIdx.x
    bw = cuda.blockDim.x
    i = tx + ty * bw
    if i<local_device_output_array.size:
        if (pivot_point - 0.05) < local_device_array[i] < (pivot_point + 0.05):
            local_device_output_array[i] = True
        else:
            local_device_output_array[i] = False


threads_per_block = 128
blocks_per_grid = (array_of_random.size + (threads_per_block - 1)) // threads_per_block

for x in range(3):

    start = time.clock()
    count_array_cuda[blocks_per_grid, threads_per_block](device_array, .5, device_output_array)
    cuda.synchronize()
    stop = time.clock()
    result = np.sum(device_output_array.copy_to_host())
    print(x)
    print(result)
    print(stop - start)
