import numpy as np
from numpy import pi as pi
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import os
import time
import pandas as pd
import itertools
import time



import PWM_actuator_functions.PWM_actuator_functions
from PWM_actuator_functions.PWM_actuator_functions import *

# dirname = "../../../Projects/PMW_robot/folder/.file.txt"
timestr = time.strftime('%Y-%m-%d--%H-%M-%S')
dirname = '../../../Projects/PMW_robot/simulation_results/' + timestr + '/'
# os.makedirs(os.path.dirname(dirname), exist_ok=True)

delta_1 = 0
delta_2 = 0
delta_3 = 0

x = 0
y = 1

# SET PARAMETERS
#n_top_of_actuator = 4
#n_bottom_of_actuator = 4
#rad = 19.180
#ang = 0.9
#ang = 0.898
#bi_directional_actuator = True
#single_output_fig = True # True = Single fig at end with all configs, False = New fig each loop 

upper = [
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
for i in reversed(upper):
	bs.append([int(not j) for j in i])
for i in bs:
	upper.append(i)
# print(upper)



# lower_actuator = []
# for i in upper_actuator:
# 	lower_actuator.append([int(not j) for j in i])
lower = upper

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

link_lengths_bottom = link_lengths_top
joint_ranges_bottom = joint_ranges_top

# assert len(upper_actuator) == len(lower_actuator), "Fail: The number of actuator sections in each configuration must be the same"
# nlines = len(upper_actuator)
# color=iter(plt.cm.cool(np.linspace(0,1,nlines)))


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
		#plt.axis('equal')

		#plt.gca().set_aspect(1)
		# 
		#filename = ''.join(str(bb) for bb in b) + '-' + ''.join(str(aa) for aa in a) + '.png'
		os.makedirs(os.path.dirname(dirname), exist_ok=True)
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
						 start_point = (0.0, 0.0),  
						 plot_crest = True, plot_convex_hull = True,
						 single_output_fig = True, subplots = True):
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
		 	  'end_to_end_angle'
		 	  ])

	if (len(link_lengths_top)==1)    : link_lengths_top *= nLinks_top 
	if (len(joint_ranges_top)==1)    : joint_ranges_top *= nLinks_top
	if (len(link_lengths_bottom)==1) : link_lengths_bottom *= nLinks_top
	if (len(joint_ranges_bottom)==1) : joint_ranges_bottom *= nLinks_top 

	upper_actuator = list(map(list, itertools.product([0, 1], repeat=nLinks_top)))
	lower_actuator = list(map(list, itertools.product([0, 1], repeat=nLinks_bottom)))


	# print("lower", lower_actuator, len(lower_actuator))
	# print("upper", upper_actuator, len(upper_actuator))
	# upper_actuator = upper
	# lower_actuator = lower
	
	nlines = len(upper_actuator)
	# color=iter(plt.cm.cool(np.linspace(0,1,nlines)))
	color=iter(plt.cm.binary(np.linspace(0.2, 1, nlines)))

	# if subplots:
	# 	nLinks = nLinks_top + nLinks_bottom
	# 	print(nLinks)
	# 	print(np.ceil((nLinks)/2))
	# 	print(int(np.ceil((nLinks)/2)))
	# 	#print(int(np.ceil((nLinks)/2)))
	# 	f, axarr = plt.subplots(2, 2)#int(np.ceil((nLinks)/2)))
	# 	subplot_idx = []
	# 	if nLinks > 2
	# 	for i in range(nLinks):
	# 		subplot_idx.append((nLinks//2, 0))
	# 		subplot_idx.append((nLinks//2, 1))


	#, vmin=0, vmax=20nLinks)/2


	if (nLinks_bottom and nLinks_top):
		bidirectional = True 
		assert (len(upper_actuator) == len(lower_actuator)), "Fail: In the current model, the number of actuator sections in each configuration must be the same or zero"
	else:
		bidirectional = False

	n_configs = np.max([len(upper_actuator), len(lower_actuator)])

	if subplots:
			print("len upper", len(upper_actuator))
			print("len lower", len(lower_actuator))
			#n_configs = np.max([len(upper_actuator), len(lower_actuator)])

			print("n configs", n_configs)
			
			if n_configs <= 2:
				print("only 2")

				#gs = gridspec.GridSpec(2, 1, wspace=0.025, hspace=0.05)
				f, axarr = plt.subplots(2, sharex=True, sharey=True)#, gridspec_kw={'wspace':0.025, 'hspace':0.05})

				subplot_idx = [0, 1]
				
			else:
				#gs = gridspec.GridSpec(int(np.ceil((n_configs)/2)), 2, wspace=0.025, hspace=0.05)
				f, axarr = plt.subplots(int(np.ceil((n_configs)/2)), 2, sharex=True, sharey=True)#, gridspec_kw={'wspace':0, 'hspace':0})
				subplot_idx = []
				print(subplot_idx)
				for i in range(int(np.ceil(n_configs/2))):
					print("appending", i)
					subplot_idx.append((i, 0))
					subplot_idx.append((i, 1))
					print(subplot_idx)

			##axarr.set(aspect=1)

	for m, (u_states, l_states) in enumerate(itertools.zip_longest(upper_actuator, lower_actuator)):

		

		# if subplots:
		# 	print("len upper", len(upper_actuator))
		# 	print("len lower", len(lower_actuator))
		# 	n_configs = np.max([len(upper_actuator), len(lower_actuator)])

		# 	print("n configs", n_configs)
			
		# 	if n_configs <= 2:
		# 		print("only 2")

		# 		f, axarr = plt.subplots(2)
		# 		subplot_idx = [0, 1]
				
		# 	else:
		# 		f, axarr = plt.subplots(2, int(np.ceil((n_configs)/2)))
		# 		subplot_idx = []
		# 		print(subplot_idx)
		# 		for i in range(int(np.ceil(n_configs/2))):
		# 			print("appending", i)
		# 			subplot_idx.append((i, 0))
		# 			subplot_idx.append((i, 1))
		# 			print(subplot_idx)






		col=next(color)

		direction_up = True
		if nLinks_top:
			tip_upper, plotted_points_upper = bistable_actuator(numLinks = nLinks_top,
			                  			  actuator_extends_up = True, # False = fixed at top of actuator
						                  	link1_fixed_fixed = True, # False = link1_fixed_free
						                  	link_states = u_states, 
						                  	link_lengths = link_lengths_top,
						                  	joint_ranges = joint_ranges_top,
						                  	draw_actuator = True,
						                  	set_plot_colour = True,
						                  	plot_colour = col)


		if nLinks_bottom:
			tip_lower, plotted_points_lower = bistable_actuator(numLinks = nLinks_bottom,
			                  			  actuator_extends_up = False, # False = fixed at top of actuator
			                  				   link1_fixed_fixed = True, # False = link1_fixed_free
			                  				   link_states = l_states, 
			                  				   link_lengths = link_lengths_bottom,
			                                   joint_ranges = joint_ranges_bottom,
			                                   draw_actuator = True,
			                                   set_plot_colour = True,
			                                   plot_colour = col)

		if bidirectional:
			end_coordinates = [tip_upper, tip_lower]
		else:
			if nLinks_top:
				end_coordinates = [tip_upper, start_point]
			else: 
				end_coordinates = [start_point, tip_lower]


		if plot_crest:
			print(axarr.shape)
			print("sp_idx" , subplot_idx[m])
			print("sp_idx_all" , subplot_idx)
			p = axarr[subplot_idx[m]] if (single_output_fig and subplots) else plt

			print(p)
			# https://stackoverflow.com/questions/14907062/aspect-ratio-in-subplots-with-various-y-axes
			p.set(aspect=1)

			p.plot([end_coordinates[0][x], end_coordinates[1][x]], 
					 [end_coordinates[0][y], end_coordinates[1][y]], 
					 c=col, linestyle='--', linewidth=0.5)


		if plot_convex_hull:
			if bidirectional:
				plotted_points = np.hstack((plotted_points_upper, plotted_points_lower))
			else:
				if nLinks_top:
					plotted_points = plotted_points_upper
				else: 
					plotted_points = plotted_points_lower

			plotted_points = plotted_points.transpose()
			
			hull = ConvexHull(plotted_points)

			p = axarr[subplot_idx[m]] if (single_output_fig and subplots) else plt
			p.set(aspect=1)

			p.fill(plotted_points[hull.vertices,0], plotted_points[hull.vertices,1], c=col, alpha=0.5)

			# get the indices of the vertices of the convex hull
			indices_x = plotted_points[hull.vertices,0]
			indices_y = plotted_points[hull.vertices,1]

			#### POTENTIALLY USEFUL STUFF ABOUT HOW TO FIND THE LONGEST SIDE OF THE HULL ####################
			
			# points_diff = np.array([indices_x[1:] - indices_x[:-1],
			# 				        indices_y[1:] - indices_y[:-1]])

			# points_diff = np.hstack((#points_diff, 
			# 			  np.array([[indices_x[0] - indices_x[-1],
			# 				    	 indices_y[0] - indices_y[-1]]]).transpose(),
			# 			  points_diff))

			# print(points_diff[:,:4])
			# print()
			# #hull_side_len = dist = np.sqrt(np.sum(diff**2,axis=1))
			# hull_side_len = points_diff**2
			# print(hull_side_len[:,:4])
			# print()

			# hull_side_len = np.sum(points_diff**2, axis=0)
			# print(hull_side_len)
			# print(hull_side_len.shape)


			# hull_side_len = np.sum(points_diff**2, axis=0)**(1/2)
			# print(hull_side_len)
			# print(hull_side_len.shape)

			# longest = np.argmax(hull_side_len)

			#################################################################################################


		actuator_length = np.sqrt(float((end_coordinates[0][x] - end_coordinates[1][x])**2 + 
			                      (end_coordinates[0][y] - end_coordinates[1][y])**2))

	
		# 	max_length = actuator_length
		
		#ND_length = actuator_length / max_length

		# ND_hypot_len = sp.symbols('ND_hypot_len')

		# # area = np.sqrt(((hypot_length / max_length)**2) - (ND_length**2)) * ND_length / 2

		# # ND_area = np.sqrt(((ND_hypot_len)**2)-(ND_length**2)) * ND_length / 2

		# ND_area = ((ND_hypot_len)**2)-(actuator_length**2)**(1/2) * actuator_length / 2








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
				fname = ''.join(str(us) for us in reversed(u_states)) + ''.join(str(ls) for ls in l_states) 
			# else:
			# 	fname = ''.join(str(upper) for upper in reversed(u)) 
			elif nLinks_top and not nLinks_bottom:
				fname = ''.join(str(us) for us in reversed(u_states))

			elif nLinks_top and not nLinks_bottom:
				fname = ''.join(str(ls) for ls in l_states)

			else:
				raise ValueError("No links to plot")
			output_figure(fname)
			

		d.append([ nLinks_top,
				   u_states[:nLinks_top] if nLinks_top else None, 
				   np.around(link_lengths_top[:nLinks_top], 2) if nLinks_top else None,
				   np.around(joint_ranges_top[:nLinks_top], 2) if nLinks_top else None,
				   nLinks_bottom,
				   np.around(l_states[:nLinks_bottom], 2) if nLinks_bottom else None,
				   np.around(link_lengths_bottom[:nLinks_bottom], 2) if nLinks_bottom else None, 
				   np.around(joint_ranges_bottom[:nLinks_bottom], 2) if nLinks_bottom else None,
				   0,
				   0,
				   [round(end_coordinates[1][0],2), round(end_coordinates[1][1],2), 
				   	round(end_coordinates[0][0],2), round(end_coordinates[0][1],2)],
				   round(actuator_length, 2),
				   round(actuator_angle, 2)
				   ])

	# sort data in order of sail angle to y datum
	# d[1:] = d[1:].sort(key=lambda x: x[12])
	d[1:] = sorted(d[1:], key=lambda x: x[12])

	

	# angular difference between each position
	#f = [[y for y in x] for x in l]
	#f = [[i for i,j in zip(di, d[0]) if j == 'end_to_end_angle'] for di in d[1:]]

	# f = [i for i,j in zip(d[1], d[0]) if j == 'end_to_end_angle']
	# # https://stackoverflow.com/questions/18072759/list-comprehension-on-a-nested-list
	# list_ang = [i for di in d[1:] for i, j in zip(di, d[0]) if j == 'end_to_end_angle'] 
	#print("list ang =", f)

	#list_ang = [i[-1] for i in d[1:]]
	# print("list ang =", list_ang)
	# diff_ang = [abs(j-i) for i,j in zip(list_ang, list_ang[1:])] 
	# mean_diff_ang = np.mean(diff_ang)
	# SD_diff_ang = np.std(diff_ang)
	# var_diff_ang = np.var(diff_ang)
	# print(list_ang)

	# print(diff_ang, mean_diff_ang, var_diff_ang, SD_diff_ang)

	linkstr = str(nLinks_top + nLinks_bottom) + 'Links-'

	# write data to file
	df = pd.DataFrame(d[1:], columns=d[0])
	# new_column = df['end_to_end_angle'] + 1
	# df['mean_diff_ang'] = new_column
	# df['mean_diff_ang'][1] = mean_diff_ang


	fname = 'data-' + linkstr + timestr + '.csv'
	os.makedirs(os.path.dirname(dirname), exist_ok=True)
	df.to_csv(dirname + fname)

	# add data from analysing data for all configs
	# https://stackoverflow.com/questions/18072759/list-comprehension-on-a-nested-list
	list_ang = [i for di in d[1:] for i, j in zip(di, d[0]) if j == 'end_to_end_angle'] 
	diff_ang = [abs(j-i) for i,j in zip(list_ang, list_ang[1:])] 
	mean_diff_ang = np.mean(diff_ang)
	SD_diff_ang = np.std(diff_ang)
	var_diff_ang = np.var(diff_ang)

	#diff_ang.append(None)
	diff_ang.insert(0, None)


	

	df = pd.read_csv(dirname + fname)
	df['diff_ang'] = diff_ang
	df['mean_diff_ang'] = mean_diff_ang
	df['SD_diff_ang'] = SD_diff_ang
	df['var_diff_ang'] = var_diff_ang
	df.to_csv(dirname + fname)


	# Single figure at the end with all configs
	if single_output_fig:
		fname = 'all_configs-' + linkstr + timestr
		output_figure(fname)

	# df = pd.DataFrame(d[1:], columns=d[0])
	# filename = 'data-' + timestr + '.csv'
	# df.to_csv(dirname + filename)
	#print(df)


# actuator_assembly(nLinks_top = 4, nLinks_bottom = 4, 
# 				  link_lengths_top = [27.0], link_lengths_bottom = [27.0],
# 				  joint_ranges_top = [pi/3], joint_ranges_bottom = [pi/3])

start = time.time()

actuator_assembly(nLinks_top = 2, nLinks_bottom = 2, 
				  link_lengths_top = [27.0], link_lengths_bottom = [27.0],
				  joint_ranges_top = [pi/3], joint_ranges_bottom = [pi/3])


