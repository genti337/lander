import math
import matplotlib.pyplot as plt

from liblander import *
from set_initial_state import *

body = DynBody()
lander = Lander()

# Set the Initial Orbital State
set_trans_state(body, frame='orbital', alt=15000.0, long_asc=0.0, inclination=math.radians(0.0), true_anom=math.radians(0.0))

# Lander Mass Properties
body.setMass(100.0)

# Initialize the Lander Object
lander.Initialize()

i = 0
pos_x = []
pos_y = []
pos_z = []
velLVLH_x = []
velLVLH_y = []
velLVLH_z = []
alt = []
time = 0.0
times = []
while i < 100000:
	lander.Update(body)
	times.append(time)
	pos_x.append(body.getECIPos(0))
	pos_y.append(body.getECIPos(1))
	pos_z.append(body.getECIPos(2))
#	velLVLH_x.append(body.getLVLHVel(0))
#	velLVLH_y.append(body.getLVLHVel(1))
#	velLVLH_z.append(body.getLVLHVel(2))
#	alt.append(body.getAlt())
#	time = time + 0.1
	i = i + 1

#plt.plot(pos_x, pos_y)
#plt.plot(alt)
#plt.show()

#fig = plt.figure()

# syntax for 3-D projection
ax = plt.axes(projection ='3d')

ax.plot3D(pos_x, pos_y, pos_z, 'green')
#ax.set_title('3D line plot geeks for geeks')
#plt.plot(times, velLVLH_x)
#plt.show()
#plt.plot(times, velLVLH_y)
#plt.show()
#plt.plot(times, velLVLH_z)
plt.show()
