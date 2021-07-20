import numpy as np
import matplotlib.pyplot as plt


test_case = [1,2,3,4,5]

x = []
y = []

chartname = f'linha_reta'

for i in test_case:
    _x = []
    _y = []

    filename = f'data/linha_reta_{i}.csv'

    with open(filename, 'r') as file:
        for line in file:
            angle, timestamp = line.rstrip().split(',')

            _y.append(int(angle))
            _x.append(float(timestamp))
    
    x.append(_x)
    y.append(_y)

fig = plt.figure()
fig.set_size_inches(3, 5, forward=True)
ax = fig.add_subplot(111)

for i in range(len(y)):
    x_distance = []

    distance = x[i][0] - x[i][-1]

    for j in x[i]:
        x_distance.append(float("{:.2f}".format((j / distance) * 2.5)))

    line, = ax.plot(y[i], x_distance, label = f'Teste {i + 1}')
    # line, = ax.plot(y[i], x[i], label = f'Teste {i + 1}')

# ax.set_ylim(0,120)

# ax.set_xlim([0, 600])
# ax.set_ylim(0,300)

ax.set_xticklabels([])

ax.set_xlim(0, 300)
ax.set_ylim(2.5, 0)


# ax.grid()

plt.legend()
plt.savefig(chartname)
