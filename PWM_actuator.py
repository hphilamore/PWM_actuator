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
dirname = '../../../Projects/PMW_robot/' + timestr + '/'
os.makedirs(os.path.dirname(dirname), exist_ok=True)

# SET PARAMETERS
#n_top_of_actuator = 4
#n_bottom_of_actuator = 4
rad = 19.180
#ang = 0.9
ang = 0.898
bi_directional_actuator = True
single_output_fig = True # True = Single fig at end with all configs, False = New fig each loop 

upper_actuator = [
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
for i in reversed(upper_actuator):
	bs.append([int(not j) for j in i])
for i in bs:
	upper_actuator.append(i)
print(upper_actuator)



# lower_actuator = []
# for i in upper_actuator:
# 	lower_actuator.append([int(not j) for j in i])
lower_actuator = upper_actuator

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

link_lengths_top = [27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 27.0]
joint_ranges_top = [pi/3, pi/3, pi/3, pi/3, pi/3, pi/3, pi/3]
joint_ranges_top = [1, 1, 1, 1, 1, 1, 1, 1]
joint_ranges_top = [1, 1, 1, 1, 1, 1, 1, 1]

link_lengths_bottom = link_lengths_top
joint_ranges_bottom = joint_ranges_top

assert len(upper_actuator) == len(lower_actuator), "Fail: The number of actuator sections in each configuration must be the same"
nlines = len(upper_actuator)
color=iter(plt.cm.cool(np.linspace(0,1,nlines)))


# data to store each time the code is run					
# d = []
# d.append(['nLinks_top',
# 		  'link_states_top',
# 		  'link_lengths_top',
# 		  'joint_ranges_top',
# 		  'nLinks_bottom',
# 		  'link_states_bottom',
# 		  'link_lengths_bottom',
# 		  'joint_ranges_bottom',
# 		  'link_offset',
# 		  'link_twist',
# 	 	  'end_coordinates',
# 	 	  'end_to_end_length', 
# 	 	  'end_to_end_angle'])

def output_figure(filename):
		plt.axis('equal')
		plt.gca().set_aspect(1)
		# 
		#filename = ''.join(str(bb) for bb in b) + '-' + ''.join(str(aa) for aa in a) + '.png'
		plt.savefig(dirname + filename + ".pdf", 
					orientation='portrait', 
					transparent=False) 
		plt.show()



link_lengths_top = [27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 27.0]
joint_ranges_top = [pi/3, pi/3, pi/3, pi/3, pi/3, pi/3, pi/3, pi/3]
joint_ranges_top = [1, 1, 1, 1, 1, 1, 1, 1]
joint_ranges_top = [1, 1, 1, 1, 1, 1, 1, 1]

link_lengths_bottom = link_lengths_top
joint_ranges_bottom = joint_ranges_top



def actuator_assembly(*, nLinks_top, nLinks_bottom, 
						 link_lengths_top = [27.0], link_lengths_bottom = [27.0],
						 joint_ranges_top = [pi/3], joint_ranges_bottom = [pi/3],
						 start_point = (0.0, 0.0)):
	"Assembles one (extending up) or two (extending down) series linked chains of bistable actuators"

	d = []
	d.append(['nLinks_top',
			  'link_states_top',
			  'link_lengths_top',
			  'joint_ranges_top',
			  'nLinks_bottom',
			  'link_states_bottom',
			  'link_lengths_bottom',
			  'joint_ranges_bottom',
			  'link_offset',
			  'link_twist',
		 	  'end_coordinates',
		 	  'end_to_end_length', 
		 	  'end_to_end_angle'])

	if (len(link_lengths_top)==1)    : link_lengths_top *= nLinks_top 
	if (len(joint_ranges_top)==1)    : joint_ranges_top *= nLinks_top
	if (len(link_lengths_bottom)==1) : link_lengths_bottom *= nLinks_top
	if (len(joint_ranges_bottom)==1) : joint_ranges_bottom *= nLinks_top 

	upper_actuator = list(map(list, itertools.product([0, 1], repeat=nLinks_top)))
	lower_actuator = list(map(list, itertools.product([0, 1], repeat=nLinks_bottom)))
	
	bidirectional = True if (nLinks_bottom and nLinks_top) else False

	for u_states, l_states in zip(upper_actuator, lower_actuator):

		c=next(color)

		direction_up = True
		if nLinks_top:
			tip_upper = bistable_actuator(numLinks = nLinks_top,
			                  			  actuator_extends_up = True, # False = fixed at top of actuator
						                  	link1_fixed_fixed = True, # False = link1_fixed_free
						                  	link_states = u_states, 
						                  	link_lengths = link_lengths_top,
						                  	joint_ranges = joint_ranges_top,
						                  	draw_actuator = True,
						                  	set_plot_colour = True,
						                  	plot_colour = c)


		if nLinks_bottom:
			tip_lower = bistable_actuator(numLinks = nLinks_bottom,
			                  			  actuator_extends_up = False, # False = fixed at top of actuator
			                  				   link1_fixed_fixed = True, # False = link1_fixed_free
			                  				   link_states = l_states, 
			                  				   link_lengths = link_lengths_bottom,
			                                   joint_ranges = joint_ranges_bottom,
			                                   draw_actuator = True,
			                                   set_plot_colour = True,
			                                   plot_colour = c)

		if bidirectional:
			end_coordinates = [tip_upper, tip_lower]
		else:
			if nLinks_top:
				end_coordinates = [tip_upper, start_point]
			else: 
				end_coordinates = [start_point, tip_lower]



		actuator_length = np.sqrt(float((end_coordinates[0][0] - end_coordinates[1][0])**2 + 
			                      (end_coordinates[0][1] - end_coordinates[1][1])**2))

		###################################
		# TODO: replace with imported function that does exactly the same but isn't working for an unknown reason
		# angle_to_Xdatum = angle_to_Xdatum(B[2], A[2], actuator_length)


		# actuator_angle = angle_to_Xdatum(end_coordinates[1], end_coordinates[0], actuator_length)
		actuator_angle = angle_to_Ydatum(end_coordinates[1], end_coordinates[0], actuator_length)
		print(actuator_angle)


		# point = B[2]
		# origin = A[2] if bi_directional_actuator else np.array([0.0, 0.0])
		# acute_angle = np.arcsin(abs( origin[y] - point[y] ) / actuator_length)
		# quadrant = np.empty((2))
		# quadrant[x] = 1 if (point[x] > origin[x]) else 0
		# quadrant[y] = 1 if (point[y] > origin[y]) else 0
		# if np.allclose(quadrant,  np.array([1, 1])):   angle = acute_angle
		# elif np.allclose(quadrant,  np.array([0, 1])): angle = pi - acute_angle
		# elif np.allclose(quadrant,  np.array([0, 0])): angle = pi + acute_angle
		# else:                                          angle = 2 * pi - acute_angle
		# angle_to_Xdatum = angle
		# ###################################

		# New figure each time code loops
		if not single_output_fig:
			if bidirectional:
				fname = ''.join(str(upper) for upper in reversed(u)) + ''.join(str() for lower in l) 
			# else:
			# 	fname = ''.join(str(upper) for upper in reversed(u)) 
			elif nLinks_top and not nLinks_bottom:
				fname = ''.join(str(upper) for upper in reversed(u))

			elif nLinks_top and not nLinks_bottom:
				fname = ''.join(str() for lower in l)

			else:

			fname += timestr
			output_figure(fname)

		d.append([ nLinks_top,
				   u_states[:nLinks_top], 
				   np.around(link_lengths_top[:nLinks_top], 2),
				   np.around(joint_ranges_top[:nLinks_top], 2),
				   nLinks_bottom,
				   np.around(l_states[:nLinks_bottom], 2),
				   np.around(link_lengths_bottom[:nLinks_bottom], 2), 
				   np.around(joint_ranges_bottom[:nLinks_bottom], 2),
				   0,
				   0,
				   [round(end_coordinates[1][0],2), round(end_coordinates[1][1],2), 
				   	round(end_coordinates[0][0],2), round(end_coordinates[0][1],2)],
				   round(actuator_length, 2),
				   round(actuator_angle, 2)
				   ])

	# sort data in order of sail angle to y datum
	d = d.sort(key=lambda x: x[12])

	# write data to file
	df = pd.DataFrame(d[1:], columns=d[0])
	filename = 'data-' + timestr + '.csv'
	df.to_csv(dirname + filename)

	# Single figure at the end with all configs
	if single_output_fig:
		fname = 'all_configs' + timestr
		output_figure(fname)

	# df = pd.DataFrame(d[1:], columns=d[0])
	# filename = 'data-' + timestr + '.csv'
	# df.to_csv(dirname + filename)
	#print(df)


actuator_assembly(nLinks_top = 6, nLinks_bottom = 6, 
				  link_lengths_top = [27.0], link_lengths_bottom = [27.0],
				  joint_ranges_top = [pi/6], joint_ranges_bottom = [pi/6])


