import matplotlib.pyplot as plt 

f = open("mobilebot-f19/pid_data/pidsmooth_0.4_20_0.txt","r")
lines = f.readlines()
# count = 1
datas = []
for line in lines:
    # print(line)
    if '|' in line and 'SENSORS' not in line and 'X' not in line:
        data = [float(i) for i in line.split("|")]
        # print(data)
        datas.append(data[0])
        # plt.plot(count, data[1])
        # count += 1
# print(datas)
plt.plot(datas)
plt.show()