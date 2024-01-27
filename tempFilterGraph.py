# Author: Andrey Dimanchev
# Graphs estimates, measurements and it compares to the truth values

import matplotlib.pyplot as plt
import numpy as np


def errorCalculation(array1, array2):
    # cast both to numpy arrays
    array1 = np.array(array1)
    array2 = np.array(array2)

    if len(array1) != len(array2):
        return "Error: Arrays must have the same length"

    difference = array1 - array2

    return difference


x = [49.986, 49.9744, 50.0136, 50.0103, 50.012, 50.0189, 50.0059, 49.9843, 49.9817, 50.0022]
y = [50.005, 49.994, 49.993, 50.001, 50.006, 49.998, 50.021, 50.005, 50, 49.997]
w = [49.986, 49.963, 50.09, 50.001, 50.018, 50.05, 49.938, 49.858, 49.965, 50.144]
z = [1,2,3,4,5,6,7,8,9,10]

error = np.abs(errorCalculation(x, y))

plt.errorbar(z, x, yerr=error, fmt='o', label='Error', capsize=5)


plt.plot(z, w, label='Measurements')
plt.plot(z,x, label='Estimated values')
plt.plot(z,y, label='Truth values')
plt.yticks(np.arange(49.90, 50.1, step=0.05))
plt.xlabel('Iteration Number')
plt.ylabel('Degrees (C)')
plt.title('Estimates vs Measurements')
plt.legend()

plt.show()
