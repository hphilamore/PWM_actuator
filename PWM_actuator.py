import numpy as np
from numpy import pi as pi
import matplotlib.pyplot as plt
import os
import time
import pandas as pd
import itertools

import PWM_actuator_functions.PWM_actuator_functions
from PWM_actuator_functions.PWM_actuator_functions import *


# dirname = "../../../Projects/PMW_robot/folder/.file.txt"
timestr = time.strftime('%Y-%m-%d--%H-%M-%S')
dirname = '../../../Projects/PMW_robot/simulation_results/' + timestr + '/'
os.makedirs(os.path.dirname(dirname), exist_ok=True)

# SET PARAMETERS
n_base_sections = 8
n_additional_sections = 8
rad = 20
#ang = 0.9
ang = pi/6
bi_directional_actuator = True
single_output_fig = True # True = Single fig at end with all configs, False = New fig each loop 

base_sections = [
				 [1, 1, 1, 1],
				 [1, 1, 1, 0],
				 [1, 1, 0, 1],
				 [1, 1, 0, 0],
				 [0, 1, 1, 1],
				 [0, 1, 1, 0],
				 [1, 0, 1, 1],
				 [1, 0, 1, 0]
				 ]

				 # [0, 1, 0, 1],
				 # [0, 1, 0, 0],
				 # [0, 1, 1, 0],
				 # [0, 1, 0, 0],
				 # [0, 0, 0, 1],
				 # [0, 0, 0, 0]
				 # ]	

bs = []
for i in reversed(base_sections):
	bs.append([int(not j) for j in i])
for i in bs:
	base_sections.append(i)
print(base_sections)



additional_sections = []
for i in base_sections:
	additional_sections.append([int(not j) for j in i])

				# [[1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1],
				#  [1, 1, 1, 1]]

# BL = [1, 0, 1, 1, 0 ,0 ,1]
# BL = [1, 0, 1, 0, 0 , 1 ,0]



base_sections = list(map(list, itertools.product([0, 1], repeat=8)))
additional_sections = list(map(list, itertools.product([0, 1], repeat=8)))

def output_figure(filename):
		plt.axis('equal')
		plt.gca().set_aspect(1)
		# 
		#filename = ''.join(str(bb) for bb in b) + '-' + ''.join(str(aa) for aa in a) + '.png'
		plt.savefig(dirname + filename + ".pdf", 
					orientation='portrait', 
					transparent=False) 
		plt.show()



assert len(base_sections) == len(additional_sections), "Fail: The number of actuator sections in each configuration must be the same"
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

	if bi_directional_actuator:
		A = actuator_1way_series(n_additional_sections, 
			                     radius = rad,
								 arc_angle = ang,
								 set_plot_colour = True,
								 plot_colour = c,
			                     actuator_base = False, 
			                     base_links = b, 
			                     addtnl_links = a, 
			                     )

	if bi_directional_actuator:
		series = b + a 
		end_coordinates = np.array([B[2], A[2]])
	else:
		series = b
		end_coordinates = np.array([B[2], np.array([0.0, 0.0])])


	actuator_length = np.sqrt((end_coordinates[0][0] - end_coordinates[1][0])**2 + 
		                      (end_coordinates[0][1] - end_coordinates[1][1])**2)

	###################################
	# TODO: replace with imported function that does exactly the same but isn't working for an unknown reason
	# angle_to_Xdatum = angle_to_Xdatum(B[2], A[2], actuator_length)
	x = 0
	y = 1
	point = B[2]
	origin = A[2] if bi_directional_actuator else np.array([0.0, 0.0])
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

	# New figure each time code loops
	if not single_output_fig:
		if bi_directional_actuator:
			fname = ''.join(str(bb) for bb in b) + '-' + ''.join(str(aa) for aa in a) 
		else:
			fname = ''.join(str(bb) for bb in b) 
		output_figure(fname)

		# plt.xlim(0, 40)
		# plt.ylim(0, 40)
		# plt.axis('equal')
		# filename = ''.join(str(bb) for bb in b) + '-' + ''.join(str(aa) for aa in a) + '.png'
		# plt.savefig(dirname + filename, 
		# 			orientation='portrait', 
		# 			transparent=False) 
		# plt.show()

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

# Single figure at the end with all configs
if single_output_fig:
	output_figure('all_configs')
	# plt.xlim(0, 40)
	# plt.ylim(0, 40)
	# plt.axis('equal')
	# filename = ''.join(str(bb) for bb in b) + '-' + ''.join(str(aa) for aa in a) + '.png'
	# plt.savefig(dirname + filename, 
	# 			orientation='portrait', 
	# 			transparent=False) 
	# plt.show()


df = pd.DataFrame(d[1:], columns=d[0])
filename = 'data.csv'
df.to_csv(dirname + filename)
print(df)





