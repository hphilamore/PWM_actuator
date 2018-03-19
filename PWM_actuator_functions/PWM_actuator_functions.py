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
    "Returns angle that the line P0-P1 (length = L) makes with the x (horizontal) datum at P0."
    x = 0
    y = 1

    acute_angle = np.arcsin(float(abs( P1[y] - P0[y] ) / L))
    print("acute angle", acute_angle)

    # if   ((P1[x] > P0[x]) & (P1[y] > P0[y])): angle = acute_angle # print("Q1")
    # elif ((P1[x] < P0[x]) & (P1[y] > P0[y])): angle = pi - acute_angle#, print("Q2")
    # elif ((P1[x] < P0[x]) & (P1[y] < P0[y])): angle = pi + acute_angle#, print("Q3")
    # else:                                     angle = 2 * pi - acute_angle#, print("Q4")

    if   ((P0[x] <= P1[x]) & (P0[y] <= P1[y])): angle = acute_angle # print("Q1")
    elif ((P0[x] > P1[x])  & (P0[y] <= P1[y])): angle = pi - acute_angle#, print("Q2")
    elif ((P0[x] > P1[x])  & (P0[y] > P1[y])):  angle = pi + acute_angle#, print("Q3")
    else: 										angle = 2 * pi - acute_angle#, print("Q4")

       
    print("angle =", angle) 
    return angle
    # quadrant = np.empty((2))
    # quadrant[x] = 1 if (ep[x] > sp[x]) else 0
    # quadrant[y] = 1 if (ep[y] > sp[y]) else 0



      
    
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

    	# if n == 1:
    	if n == 0:

    		# fixed end 
    		if link1_fixed_fixed:
    			angle = pi/2 if actuator_extends_up else (-pi/2)
    			
    		# free end
    		else:
    			angle += pi/2 if actuator_extends_up else (-pi/2)

    		H = Ai.subs({a:ai, theta:ti})
    		arc_start = SP



    	else:
    		# the offset angle due to angle of tip of previous link 
    		# if actuator_extends_up:
    		# 	angle += joint_range/2 if link_states[n-1] else (-joint_range/2)
    		# else:
    		# 	angle -= joint_range/2 if link_states[n-1] else (-joint_range/2)
    		angle += joint_range/2 if link_states[n-1] else (-joint_range/2)
    		
    		H *= Ai.subs({a:ai, theta:ti})

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

		    # if actuator_extends_up:
		    # 	origin = (arc_start[x] + radius * np.cos(origin_angle),
		    # 		  arc_start[y] + radius * np.sin(origin_angle))
		    # else:
		    # 	origin = (arc_start[x] + radius * np.cos(origin_angle),
		    # 		  arc_start[y] + radius * np.sin(origin_angle))



		    # origin = np.array([arc_start[x] + radius * np.cos(origin_angle, arc_start[y] + radius * np.sin(origin_angle)])

		    # print ()
		    

		    
		    # # # plot arc
		    # print(origin)
		    plot_start_angle = angle_to_Xdatum(origin, arc_start, radius)
		    # plot_end_angle = angle_to_Xdatum(origin, arc_end, radius)
		    plot_end_angle = plot_start_angle + (joint_range if bend_CCW else (-joint_range))
		    # arc_end_to_origin = angle_to_Xdatum(origin, arc_end, radius)

		    # deal with case where arc crosses from 4th to 1st quadrant
		    # if bend_CCW:
		    # 	if plot_start_angle > plot_end_angle:
		    # 		plot_start_angle = -1 * (2 * pi - (plot_start_angle))
		    # else:
		    # 	if plot_start_angle < plot_end_angle:
		    # 		plot_start_angle = -1 * (2 * pi - (plot_start_angle))

		        
		    arc_points = np.linspace(plot_start_angle, 
		    						 plot_end_angle, 
		    						 num_plot_points) 

		    # print(origin[x] + radius)       

		    arc = np.array([origin[x] + radius * np.cos(arc_points), 
		                    origin[y] + radius * np.sin(arc_points)])



		    # draw arc
		    # # PLOT: 
		   	# # ARC
		    if set_plot_colour:
		    	#plt.plot(arc[x], arc[y], color=plt.cm.cool(plot_colour))
		    	plt.plot(arc[x], arc[y], c=plot_colour)  
		    else:
		    	plt.plot(arc[x], arc[y])

		    plt.plot(origin[x], origin[y], 'co')
		    plt.plot(arc[x, 0], arc[y, 0], 'ro')
		    plt.plot(arc_start[x], arc_start[y], 'g^')
		    plt.plot(arc[x, -1], arc[y, -1], 'bo')
		    plt.plot(arc_end[x], arc_end[y], 'm^')

		    # # start point
		    # if n == 0: 	
		    # 	if actuator_extends_up:
		    # 		plt.plot(arc[x, 0], arc[y, 0], 'ro')
		    # 		plt.plot(arc[x, 0], arc[y, 0], 'ro')
		    # 		plt.plot(arc[x, 0], arc[y, 0], 'ro')
		    # 		plt.plot(arc[x, 0], arc[y, 0], 'ro')

		    # 	else:
		    # 		plt.plot(arc[x, 0], arc[y, 0], 'g^')
		    # else:
		    # 	plt.plot(arc[x, 0], arc[y, 0], 'ko')


		    #print()
		    #print("link:" , (n+1))
		    print("SP:", SP)
		    print("EP:", EP)
		    print("arc start:", arc_start)
		    print("arc end:", arc_end)
		    print("origin", origin)   
		    print("joint_angles:", joint_angles)
		    print("radius", radius)
		    print("cord angle", cord_angle)
		    print("j:", j)
		    print("origin angle", origin_angle)
		    print("plot start angle", plot_start_angle)
		    print("plot end angle", plot_end_angle)

		    print(H)
		    #print("angle end to x datum:" , arc_start_to_x)
		    print()

	    	

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

    plt.gca().set_aspect(1)

    #plt.axis('equal')
	
    #plt.show()

    return EP

    

#EP = bistable_actuator(numLinks = 5)           
#print(EP)





