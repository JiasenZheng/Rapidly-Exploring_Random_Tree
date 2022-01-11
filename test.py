import matplotlib.pyplot as plt
import numpy as np

def f(t):
    x=np.random.rand(1)
    y=np.random.rand(1)
    return x,y

fig, ax = plt.subplots()
ax.set_xlim(0,1)
ax.set_ylim(0,1)
for t in range(10):
    x,y = f(t)
    # # optionally clear axes and reset limits
    # plt.gca().cla() 
    # ax.set_xlim(0,1)
    # ax.set_ylim(0,1)
    ax.plot(x, y, marker="s")
    ax.set_title(str(t))
    fig.canvas.draw()
    plt.pause(0.1)

plt.show()