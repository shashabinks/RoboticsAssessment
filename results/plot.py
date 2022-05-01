import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

x1 = np.array([1 ,2 ,3])
y1 = np.array([492, 1281, 2347, 3748, 5633])
y2 = np.array([293, 647, 1059, 1580, 2132])
y3 = np.array([237, 616, 1312, 2337, 3283])







plt.plot(x1, y1, label="None")
plt.plot(x1, y2, label="Basic")
plt.plot(x1, y3, label="Advanced")




plt.xlabel("Number of e-pucks", fontsize=15)
plt.ylabel("Number of collisions", fontsize=15)
plt.legend()



plt.savefig('graph.png', dpi=300)
plt.show()