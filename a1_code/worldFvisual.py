import matplotlib.pyplot as plt 
import re
import numpy as np

name = 'wfpose'
f = open('' + name + '.txt','r')
lines = f.readlines()
x = []
y = []
right_wall_dist = []
for line in lines:
    # print(line)
    if 'world frame pose:' in line:
        name, xvalue, yvalue = re.split(', |: ',line)
        x.append(float(xvalue))
        y.append(float(yvalue))
    elif 'Dist 0 =' in line:
        name, value = re.split(' = ',line)
        right_wall_dist.append(float(value))

plt.plot(right_wall_dist)
plt.yticks(np.arange(0, 2, 0.1)) 
plt.show()

plt.scatter(x, y, color='blue')
plt.title('world frame trajectory')
plt.xlabel('x')
plt.ylabel('y')
plt.xticks(np.arange(0, 2, 0.1)) 
plt.yticks(np.arange(0, 2, 0.1)) 
plt.savefig(name + ".png")