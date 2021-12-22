import math
import matplotlib.pyplot as plt

from liblander import *
from set_initial_state import *

body = DynBody()
eom = EoM()

# Set the Initial Orbital State
set_trans_state(body, frame='orbital', alt=15000.0, long_asc=0.0, inclination=math.radians(30.0), true_anom=math.radians(0.0))

eom.Initialize()

i = 0
pos_x = []
pos_y = []
pos_z = []
alt = []
while i < 100000:
	eom.Update(body)
	pos_x.append(body.getECIPos(0))
	pos_y.append(body.getECIPos(1))
	pos_z.append(body.getECIPos(2))
	alt.append(body.getAlt())
	i = i + 1

#plt.plot(pos_x, pos_y)
#plt.plot(alt)
#plt.show()

fig = plt.figure()

# syntax for 3-D projection
ax = plt.axes(projection ='3d')

ax.plot3D(pos_x, pos_y, pos_z, 'green')
ax.set_title('3D line plot geeks for geeks')
plt.show()
