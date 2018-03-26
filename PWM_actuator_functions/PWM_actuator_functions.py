import numpy as np
from numpy import pi as pi
import matplotlib.pyplot as plt
import sympy as sp
# initialize printing so that all of the mathematical expressions can be rendered in standard mathematical notation
# from sympy.physics.vector import init_vprinting
# init_vprinting(use_latex='mathjax', pretty_print=False)
#from sympy.physics.mechanics import dynamicsymbols, Point, ReferenceFrame



def angle_to_Xdatum(P0, P1, L):
	# TODO : L should be calculated from points P0 and P1 to avoid error
	# assert can be used to check answer if desired
    "Returns angle that the line P0-P1 (length = L) makes with the x (horizontal) datum."
    x = 0
    y = 1

    # Round P0 and P1 to avoid invalid value enter to arcsin
    P0y = round(P0[y],4)
    P1y = round(P1[y],4)

    #acute_angle = np.arcsin(float(abs( P1[y] - P0[y] ) / L))
    acute_angle = np.arcsin(float(abs( P1y - P0y ) / L))
    print("acute angle", acute_angle)

    # if symmetric:
    # 	angle = acute_angle if (P0[y] <= P1[y]) else (-acute_angle)# print("Q1")

  
    if   ((P0[x] <= P1[x]) & (P0[y] <= P1[y])): angle = acute_angle # print("Q1")
    elif ((P0[x] > P1[x])  & (P0[y] <= P1[y])): angle = pi - acute_angle#, print("Q2")
    elif ((P0[x] > P1[x])  & (P0[y] > P1[y])):  angle = pi + acute_angle#, print("Q3")
    else: 										angle = 2 * pi - acute_angle#, print("Q4")

    print("angle =", angle) 
    return angle

def angle_abt_centre(P0, P1, L):
	# TODO : L should be calculated from points P0 and P1 to avoid error
	# assert can be used to check answer if desired
    "Returns angle (+ve, -ve) that the line P0-P1 (length = L) makes with the x (horizontal) datum."
    x = 0
    y = 1

    # Round P0 and P1 to avoid invalid value enter to arcsin
    P0y = round(P0[y],4)
    P1y = round(P1[y],4)

    acute_angle = np.arcsin(float(abs( P1y - P0y ) / L))
    print("acute angle", acute_angle)

    if   ((P0[x] <= P1[x]) & (P0[y] <= P1[y])): angle =   acute_angle # print("Q1")
    elif ((P0[x] > P1[x])  & (P0[y] <= P1[y])): angle =   pi - acute_angle#, print("Q2")
    elif ((P0[x] > P1[x])  & (P0[y] > P1[y])):  angle = -(pi - acute_angle)#, print("Q3")
    else: 										angle = - acute_angle#, print("Q4")

    print("actuator angle =", angle) 
    return angle



def angle_to_Ydatum(P0, P1, L, symmetric = True):
	# TODO : L should be calculated from points P0 and P1 to avoid error
	# assert can be used to check answer if desired
    "Returns angle (+ve, -ve) that the line P0-P1 (length = L) makes with the y (vertical) datum."
    x = 0
    y = 1

    # Round P0 and P1 to avoid invalid value enter to arcsin
    P0x = round(P0[x],4)
    P1x = round(P1[x],4)

    acute_angle = np.arcsin(float(abs( P1x - P0x ) / L))
    print("acute angle", acute_angle)

    if symmetric:

    	angle = acute_angle if (P0[x] >= P1[x]) else (-acute_angle)# print("Q1")

    # else:
    # 	if   ((P0[y] <= P1[y]) & (P0[x] <= P1[x])): angle = acute_angle # print("Q1")
	   #  elif ((P0[y] > P1[y])  & (P0[x] <= P1[x])): angle = pi - acute_angle#, print("Q2")
	   #  elif ((P0[y] > P1[y])  & (P0[x] > P1[x])):  angle = pi + acute_angle#, print("Q3")
	   #  else: 										angle = 2 * pi - acute_angle#, print("Q4")

       
    print("angle =", angle) 
    return angle

# def sym_angle_to_Xdatum(P0, P1, L):
# 	# TODO : L should be calculated from points P0 and P1 to avoid error
# 	# assert can be used to check answer if desired
#     "Returns angle (+ve, -ve) that the line P0-P1 (length = L) makes with the y (horizontal) datum."
#     x = 0
#     y = 1

#     # Round P0 and P1 to avoid invalid value enter to arcsin
#     P0y = round(P0[y],4)
#     P1y = round(P1[y],4)

#     acute_angle = np.arcsin(float(abs( P1y - P0y ) / L))
#     print("acute angle", acute_angle)

#     angle = acute_angle if (P0[x] >= P1[x]) else (-acute_angle)# print("Q1")
       
#     print("angle =", angle) 
#     return angle



      
    
def bistable_actuator(*, numLinks = 2,
                         #*, 
                         actuator_extends_up = False, # False = fixed at top of actuator
                         link1_fixed_fixed = True, # False = link1_fixed_free
                         start_point = (0,0),
                         #radius = 27, 
                         link_states = [1, 1, 0, 1, 0, 1 ,0], 
                         link_lengths = [27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 27.0],
                         joint_ranges = [pi/3, pi/3, pi/3, pi/3, pi/3, pi/3, pi/3],
                         link_offset = 0,
                         link_twist = 0,
                         #addtnl_links = [0, 1, 0, 1, 0, 1 ,0], 
#                          arc_angle = 0.93, 
                         # arc_angle = 0.6,
                         #arc_angle = 0.898,
                         #COA = [0.0, 0.0],
                         draw_actuator = True,
                         set_plot_colour = False,
                         plot_colour = 'k'): 
    "Plots 2D bistable actuators in a linked unidirectional series from a vertically aligned start section"
    
    num_plot_points = 1000
    x = 0
    y = 1


    SP = sp.Matrix([start_point[0],
					start_point[1],
					0,
					1])

    aList = sp.symbols('a0:%d'%numLinks)
    tList = sp.symbols('theta0:%d'%numLinks)
    theta, a, alpha, d = sp.symbols('theta, a, alpha, d')
    theta, a, alpha, d

    rot = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0], 
		             [sp.sin(theta),  sp.cos(theta), 0], 
		             [0,              0,             1]])


    trans = sp.Matrix([a * sp.cos(theta),
	                   a * sp.sin(theta),
	                   d])

    last_row = sp.Matrix([[0, 0, 0, 1]])

    Ai = sp.Matrix.vstack(sp.Matrix.hstack(rot, trans), last_row)


    # each joint angle (relative to previous link)
    joint_angles = []



    # for n, (state, joint_range, length) in enumerate(zip(link_states, joint_ranges, link_lengths), 1):
    for n, (state, joint_range, length, ai, ti) in enumerate(zip(link_states[:numLinks], 
    															 joint_ranges[:numLinks], 
    															 link_lengths[:numLinks], 
    															 aList[:numLinks], 
    															 tList[:numLinks])):
    	bend_CCW = state

    	angle = joint_range/2 if bend_CCW else (-joint_range/2)



    	print()
    	print("link:" , (n+1))
    	print("upper:" , actuator_extends_up)
    	print(link_states)
    	
    	# if n == 1:
    	if n == 0:

    		# fixed end 
    		if link1_fixed_fixed:
    			angle = pi/2 if actuator_extends_up else (-pi/2)
    			
    		# free end
    		else:
    			angle += pi/2 if actuator_extends_up else (-pi/2)

    		H = Ai.subs({a:ai, theta:ti})

    		if draw_actuator:
    			arc_start = SP



    	else:
    		# the offset angle due to angle of tip of previous link 
    		# if actuator_extends_up:
    		# 	angle += joint_range/2 if link_states[n-1] else (-joint_range/2)
    		# else:
    		# 	angle -= joint_range/2 if link_states[n-1] else (-joint_range/2)
    		angle += joint_range/2 if link_states[n-1] else (-joint_range/2)
    		
    		H *= Ai.subs({a:ai, theta:ti})

    		if draw_actuator:
    			arc_start = arc_end

    	joint_angles.append(angle)



    	if draw_actuator:
	    	
	    	# simplify terms of matrix
		    for Y in range(Ai.shape[0]):
		    	for X in range(Ai.shape[1]):
		    		H[Y,X] = sp.trigsimp(H[Y,X].simplify())

			# substitute numerical values to get end point at each link to plot the actuator
		    Hn = H.subs({d:link_offset, alpha:link_twist})

		    for ai, ti, a_val, t_val in zip(aList[:numLinks], 
		    	                            tList[:numLinks], 
		    	                            link_lengths[:numLinks], 
		    	                            joint_angles[:numLinks]):
		    	Hn = Hn.subs({ai:a_val, ti:t_val})

		    EP = Hn * SP
		    arc_end = EP

		    # find arc radius
		    radius = length / (2 * sp.sin(joint_range/2))
		    
		    # find origin of arc
		    cord_angle = angle_to_Xdatum(arc_start, arc_end, length)
		    j = (pi - joint_range) / 2  # 1 of the 2 identical angles of the arc segment isoceles
		    

		    origin_angle = cord_angle + (j if bend_CCW else (-j))

		    origin = (arc_start[x] + radius * np.cos(origin_angle),
		    		  arc_start[y] + radius * np.sin(origin_angle))

		    plot_start_angle = angle_to_Xdatum(origin, arc_start, radius)

		    plot_end_angle = plot_start_angle + (joint_range if bend_CCW else (-joint_range))
		        
		    arc_points = np.linspace(plot_start_angle, 
		    						 plot_end_angle, 
		    						 num_plot_points)      

		    arc = np.array([origin[x] + radius * np.cos(arc_points), 
		                    origin[y] + radius * np.sin(arc_points)])


		    # complile a single array of all points plotted to use for convex hull
		    if n == 0:
		    	all_arc_points = arc

		    else:
		    	all_arc_points = np.hstack((all_arc_points, arc))
		    # draw arc
		    # # PLOT: 
		   	# # ARC
		    #plt.plot(arc[x], arc[y], c=plot_colour if set_plot_colour else 'Auto')  


    	else:
    		all_arc_points = None		
	    	

    # simplify terms of matrix
    for Y in range(Ai.shape[0]):
    	for X in range(Ai.shape[1]):
    		H[Y,X] = sp.trigsimp(H[Y,X].simplify())


	# substitute in numerical values
    Hn = H.subs({d:link_offset, alpha:link_twist})

    for ai, ti, a_val, t_val in zip(aList[:numLinks], 
    	                            tList[:numLinks], 
    	                            link_lengths[:numLinks], 
    	                            joint_angles[:numLinks]):
    	Hn = Hn.subs({ai:a_val, ti:t_val})

    SP = sp.Matrix([start_point[0],
					start_point[1],
					0,
					1])

    EP = Hn * SP

    
    #plt.gca().set_aspect(1)

    #plt.axis('equal')
	
    #plt.show()

    return EP, all_arc_points

    

#EP = bistable_actuator(numLinks = 5)           
#print(EP)





