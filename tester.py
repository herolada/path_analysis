import matplotlib.pyplot as plt
import numpy as np

fig,ax = plt.subplot()
x = np.array([0,2,2,0])
y = np.array([0,0,2,2])

ax.fill(x,y,c='#FF0000', alpha=0.2, zorder = 2, fill=False, hatch = '/')