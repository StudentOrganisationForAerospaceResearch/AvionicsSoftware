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


x = [50.486, 50.9349, 51.5579, 51.9748, 52.4859, 53.0167, 53.4131, 53.8317, 54.4276, 55.0735] #estimates 
y = [50.505,50.994,51.493,52.001,52.506,52.998,53.521,54.005,54.5, 54.997] #truths 
w = [50.486,50.963,51.597,52.001,52.518,53.05,53.438,53.858,54.465,55.114] #measurements 
z = [1,2,3,4,5,6,7,8,9,10]

error = np.abs(errorCalculation(x, y))

plt.errorbar(z, x, yerr=error, fmt='o', label='Error', capsize=5)


plt.plot(z, w, label='Measurements')
plt.plot(z,x, label='Estimated values')
plt.plot(z,y, label='Truth values')
# plt.yticks(np.arange(49.90, 50.1, step=0.05))
plt.xlabel('Iteration Number')
plt.ylabel('Degrees (C)')
plt.title('Estimates vs Measurements')
plt.legend()

plt.show()
