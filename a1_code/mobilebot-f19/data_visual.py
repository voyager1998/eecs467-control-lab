import matplotlib.pyplot as plt 

name = 'step_0.4_2_0'
f = open('mobilebot-f19/pid_data/' + name + '.txt','r')
lines = f.readlines()
count = 1
left = []
right = []
time = []
for line in lines:
    # print(line)
    if '|' in line and 'SENSORS' not in line and 'X' not in line:
        data = [float(i) for i in line.split("|")]
        # print(data)
        left.append(data[2])
        right.append(data[3])
        time.append(count * 0.05)
        count += 1
plt.plot(time, left, color='blue')
plt.title('Left Wheel')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.savefig("mobilebot-f19/pid_data/" + name + "_left.png")

plt.plot(time, right, color='red')
plt.title('Right Wheel')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.savefig("mobilebot-f19/pid_data/" + name + "_right.png")
