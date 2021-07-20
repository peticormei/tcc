import numpy as np
import matplotlib.pyplot as plt


test_case = [1,2,3,4,5]

x = []
y = []

for i in test_case:
    # chartname = f'pivot_90_turn_chart_{i}'
    # filename = f'data/pivot_90_turn_{i}.csv'

    # chartname = f'pivot_180_turn_chart_{i}'
    # filename = f'data/pivot_180_turn_{i}.csv'

    chartname = f'90_turn_chart_{i}'
    filename = f'data/90_turn_{i}.csv'

    # chartname = f'180_turn_chart_{i}'
    # filename = f'data/180_turn_{i}.csv'
    _x = []
    _y = []

    with open(filename, 'r') as file:
        for line in file:
            angle, timestamp = line.rstrip().split(',')

            _y.append(int(angle))
            _x.append(float(timestamp))

    x.append(_x)
    y.append(_y)

fig = plt.figure()

ax = fig.add_subplot(111)

for i in range(len(y)):
    line, = ax.plot(x[i], y[i], label = f'Teste {i + 1}')

# ymax = max(y)
# xpos = y.index(ymax)
# xmax = x[xpos]

# ax.annotate(y[0], xy=(x[0], y[0]), xytext=(x[0], y[0]+1))
# ax.annotate(y[-1], xy=(x[-1], y[-1]), xytext=(x[-1], y[-1]+1))

# ax.annotate(ymax, xy=(xmax, ymax), xytext=(xmax, ymax+1))

ax.set_ylim(0,110)
# ax.set_ylim(0,220)

ax.grid()



plt.ylabel('Ângulo')
plt.xlabel('Tempo (Segundos)')

plt.legend()
plt.figure(figsize=(1,1))
plt.savefig(chartname)
