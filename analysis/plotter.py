import os
import pandas as pd
import matplotlib.pyplot as plt

# some constants
res_path = '/home/flora/mdmi_data/'
state_key = ['t', 'x', 'y', 'z']


# find out players recorded, and sort by their id
players = [p for p in next(os.walk(res_path))[1]]
states = {p:None for p in players}
cmds = {p:None for p in players}
defenders = sorted([p for p in players if 'D' in p], key=lambda x: int(x[1:]))
intruders = sorted([p for p in players if 'I' in p], key=lambda x: int(x[1:]))

for p in states:
	data = pd.read_csv(res_path + p + '/State.csv')
	states[p] = {k:data[k].to_numpy() for k in state_key}

for p, s in states.items():
	c = 'r' if 'I' in p else 'b'
	plt.plot(s['x'], s['y'], color=c, label=p)


# fs = 14
plt.axis("equal")
plt.grid()
plt.show()

# '/home/flora/mdmi_data/'+str(self)