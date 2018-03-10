import numpy as np
from numpy import pi as pi
import matplotlib.pyplot as plt
import os
import time
import pandas as pd

import PWM_actuator_functions.PWM_actuator_functions
from PWM_actuator_functions.PWM_actuator_functions import *

# dirname = "../../../Projects/PMW_robot/folder/.file.txt"
timestr = time.strftime('%Y-%m-%d--%H-%M-%S')
dirname = '../../../Projects/PMW_robot/' + timestr + '/'
os.makedirs(os.path.dirname(dirname), exist_ok=True)

n_base_sections = 4
n_additional_sections = 4
rad = 27
ang = 0.6


base_sections = [[1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1]]

additional_sections = [[1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1],
				 [1, 1, 1, 1]]

# BL = [1, 0, 1, 1, 0 ,0 ,1]
# BL = [1, 0, 1, 0, 0 , 1 ,0]

assert len(base_sections) == len(additional_sections), "Fail: The number of configurations of each actuator section must be the same"
#color=iter(plt.cm.rainbow(np.linspace(0,1,len(base_sections))))
nlines = len(base_sections)
#colour_idx = np.linspace(0, 1, n_lines)
color=iter(plt.cm.cool(np.linspace(0,1,nlines)))
					
d = []
d.append(['radius', 
	 'arc_angle', 
	 'series', 
	 'n_base',
	 'n_addtnl',
	 'end_coordinates',
	 'end_to_end_length', 
	 'end_to_end_angle'])



# TODO : OOP
# TODO : encapsulate in function

for b, a in zip(base_sections, additional_sections):

	c=next(color)

	B = actuator_1way_series(n_base_sections, 
							 radius = rad,
							 arc_angle = ang, 
							 set_plot_colour = True,	
							 plot_colour = c,	
							 actuator_base = True, 
							 base_links = b)

	A = actuator_1way_series(n_additional_sections, 
		                     radius = rad,
							 arc_angle = ang,
							 set_plot_colour = True,
							 plot_colour = c,
		                     actuator_base = False, 
		                     base_links = b, 
		                     addtnl_links = a, 
		                     )

	series = b + a

	print(f"end coordinates {B[2], A[2]}")
	end_coordinates = np.array([B[2], A[2]])

	# actuator_length = np.sqrt((B[2][0]-A[2][0])**2 + 
	# 	                      (B[2][1]-A[2][1])**2)

	actuator_length = np.sqrt((end_coordinates[0][0] - end_coordinates[1][0])**2 + 
		                      (end_coordinates[0][1] - end_coordinates[1][1])**2)

	###################################
	# TODO: replace with imported function that does exactly the same but isn't working
	# angle_to_Xdatum = angle_to_Xdatum(B[2], A[2], actuator_length)
	x = 0
	y = 1
	point = B[2]
	origin = A[2]
	acute_angle = np.arcsin(abs( origin[y] - point[y] ) / actuator_length)
	quadrant = np.empty((2))
	quadrant[x] = 1 if (point[x] > origin[x]) else 0
	quadrant[y] = 1 if (point[y] > origin[y]) else 0
	if np.allclose(quadrant,  np.array([1, 1])):   angle = acute_angle
	elif np.allclose(quadrant,  np.array([0, 1])): angle = pi - acute_angle
	elif np.allclose(quadrant,  np.array([0, 0])): angle = pi + acute_angle
	else:                                          angle = 2 * pi - acute_angle
	angle_to_Xdatum = angle
	###################################


	plt.xlim(0, 40)
	plt.ylim(0, 40)
	plt.axis('equal')
	filename = ''.join(str(bb) for bb in b) + '-' + ''.join(str(aa) for aa in a) + '.png'
	plt.savefig(dirname + filename, 
				orientation='portrait', 
				transparent=False) 
	plt.show()

	d.append([ rad, 
			   ang,
			   series,
			   n_base_sections, 
			   n_additional_sections,
			   end_coordinates,
			   # [[np.round(end_coordinates[0][0],2), np.round(end_coordinates[0][1],2)]
			   # 	[np.round(end_coordinates[1][0],2), np.round(end_coordinates[1][1],2)]],	
			   np.round(actuator_length, 2),
			   np.round(angle_to_Xdatum, 2)
			   ])

df = pd.DataFrame(d[1:], columns=d[0])
filename = 'data.csv'
df.to_csv(dirname + filename)
print(df)





