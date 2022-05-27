
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots(figsize=(12,12), dpi=200)
x = np.array([0,2,2,0])
y = np.array([0,0,2,2])

plt.rcParams['hatch.linewidth'] = 10  # previous pdf hatch linewidth

ax.fill(x,y,c='#FF0000', alpha=0.8, zorder = 2, fill=False, hatch = '/')
plt.show()