import numpy as np
from numpy import pi as pi
import matplotlib.pyplot as plt
import sympy as sp
# initialize printing so that all of the mathematical expressions can be rendered in standard mathematical notation
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)
#from sympy.physics.mechanics import dynamicsymbols, Point, ReferenceFrame

# bi-state 0 = bend to left --> draw link by moving through NEGATIVE angular displacement in regular cartesion FOR
# bi_state 1 = bend to right --> draw link by moving through POSITIVE angular displacement in regular cartesian FOR
# COA = central point of actuator 

def angle_to_Xdatum(point, o, r):
    "Returns angle between point-origin and 1st quadrant horizontal datum"
    x = 0
    y = 1
    origin = o
    radius = r
    acute_angle = np.arcsin(abs( origin[y] - point[y] ) / radius)
    quadrant = np.empty((2))
    quadrant[x] = 1 if (point[x] > origin[x]) else 0
    quadrant[y] = 1 if (point[y] > origin[y]) else 0

    if np.allclose(quadrant,  np.array([1, 1])):   angle = acute_angle
    elif np.allclose(quadrant,  np.array([0, 1])): angle = pi - acute_angle
    elif np.allclose(quadrant,  np.array([0, 0])): angle = pi + acute_angle
    else:                                          angle = 2 * pi - acute_angle
        
    return angle

      
    
def plot_bistable_actuator(numLinks,
                         *, 
                         fixed_at_bottom = True, # False = fixed at top of actuator
                         link1_fixed_fixed = True, # False = link1_fixed_free
                         start_point = (0,0),
                         radius = 27, 
                         link_states = [1, 1, 0, 1, 0 , 1 ,0], 
                         link_lengths = [27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 27.0],
                         joint_ranges = [pi/3, pi/3, pi/3, pi/3, pi/3, pi/3, pi/3],
                         link_offset = 0,
                         link_twist = 0,
                         addtnl_links = [0, 1, 0, 1, 0, 1 ,0], 
#                          arc_angle = 0.93, 
                         # arc_angle = 0.6,
                         #arc_angle = 0.898,
                         #COA = [0.0, 0.0],
                         set_plot_colour = False,
                         plot_colour = 'k'): 
    "Plots 2D bistable actuators in a linked unidirectional series from a vertically aligned start section"
    
    numsegments = 1000
    x = 0
    y = 1

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
    	
    	angle = joint_range/2 if state else (-joint_range/2)

    	# if n == 1:
    	if n == 0:

    		# fixed end 
    		if link1_fixed_fixed:
    			angle = pi/2 
    			
    		# free end
    		else:
    			angle += pi/2

    		H = Ai.subs({a:ai, theta:ti})


    	else:
    		# the offset angle due to angle of tip of previous link 
    		angle += joint_range/2 if link_states[n-1] else (-joint_range/2)
    		H *= Ai.subs({a:ai, theta:ti})

    	joint_angles.append(angle)
    	
    	# 
    	# H *= Ai.subs({a:ai, theta:ti})

    for y in range(Ai.shape[0]):
    	for x in range(Ai.shape[1]):
    		H[y,x] = sp.trigsimp(H[y,x].simplify())

    H

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

    return EP

    
EP = plot_bistable_actuator(2,

                         fixed_at_bottom = True, # False = fixed at top of actuator
                         link1_fixed_fixed = False, # False = link1_fixed_free
                         start_point = (0,0),
                         radius = 27, 
                         link_states = [0, 0, 0, 1, 0 , 1 ,0], 
                         link_lengths = [27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 27.0],
                         joint_ranges = [pi/3, pi/3, pi/3, pi/3, pi/3, pi/3, pi/3],
                         link_offset = 0,
                         link_twist = 0,
                         addtnl_links = [0, 1, 0, 1, 0, 1 ,0], 
#                          arc_angle = 0.93, 
                         # arc_angle = 0.6,
                         #arc_angle = 0.898,
                         #COA = [0.0, 0.0],
                         set_plot_colour = False,
                         plot_colour = 'k')
             
print(EP)








    		
#     		if link1_fixed_fixed:
#     			pass

#     		else:
    			







    
#     # the bistable state of the link
#     #link_states = link_states if fixed_at_bottom else addtnl_links
#     state = link_states[n-1] 
#     print(f"state {state}")



#     # the state of the first link in the actuator (all others connect to) 
#     #central_link_state = link_states[0]
    
#     # DEFINE FIRST LINK
#     # first link in series (base or additional) 
#     if n == 1:        
#         if central_link_state:
#             start_angle = (0 - (arc_angle / 2)) 
#         else:
#             start_angle = (pi + (arc_angle / 2))
        

#         # define the ORIGIN for vertically aligned arc
#         sagitta = radius * (1 - np.cos(arc_angle / 2))
#         origin = np.array(COA)
#         origin[x] += (radius - sagitta) * (-1 if central_link_state else 1)
        

#         # # define the START POINT of the arc
#         # start_point = origin + np.array([radius * np.cos(start_angle),                                             
#         #                                  radius * np.sin(start_angle)]) 


#         # # re-centre the start point at (0,0)
#         plt.plot(start_point[x], start_point[y], 'mo')
#         plt.plot(origin[x], origin[y], 'mo')
#         offset = start_point
#         start_point, origin = start_point - offset, origin - offset
#         # origin -= offset
#         plt.plot(start_point[x], start_point[y], 'cx')
#         plt.plot(origin[x], origin[y], 'cx')
        

#         if not fixed_at_bottom:

#             # FLIP LINK DIRECTION IF NECESSARY 
#             if state != central_link_state:
#                 # ...mirror ORIGIN of the arc in tangent to start point
#                 origin = start_point + np.array([radius * np.cos(start_angle),                                             
#                                                  radius * np.sin(start_angle)])                
#                 # ...redefine START ANGLE
#                 start_angle = angle_to_Xdatum(start_point, origin, radius)
     
    
#     # recursively define links after first link           
#     else: 
#         AB = fixed_at_bottom
#         R = radius
#         BL = link_states
#         AL = addtnl_links
#         AA = arc_angle
#         C = COA
#         SPC = set_plot_colour
#         PC = plot_colour
#         # angle of start of arc relative to horizontal datum
#         start_angle, origin, start_point = actuator_1way_series(n-1, 
#                                                                 fixed_at_bottom = AB, 
#                                                                 radius = R,
#                                                                 link_states = BL,
#                                                                 addtnl_links = AL,
#                                                                 arc_angle = AA,
#                                                                 COA = C,
#                                                                 set_plot_colour = SPC,
#                                                                 plot_colour = PC)        
        
        
        
#         # FLIP LINK DIRECTION IF NECESSARY 
#         if link_states[n-1]!=link_states[n-2]:
#            # ...mirror ORIGIN of the arc in tangent to start point
#             origin = start_point + np.array([radius * np.cos(start_angle),                                             
#                                              radius * np.sin(start_angle)])
#             # ...redefine START ANGLE
#             start_angle = angle_to_Xdatum(start_point, origin, radius)
            
    
#     print(f"start_angle {start_angle} \norigin {origin} \nstart_point {start_point}")
    

#     # DEFINE ARC OF LINK
#     if fixed_at_bottom:
#     	# draw links in upward direction
#         arc_angles = start_angle + (arc_angle * (1 if state else -1))
#     else:
#     	# draw links in downward direction
#         arc_angles = start_angle + (arc_angle * (-1 if state else 1))
        
#     arc_points = np.linspace(start_angle, arc_angles, numsegments)        

#     arc = np.array([origin[x] + radius * np.cos(arc_points), 
#                     origin[y] + radius * np.sin(arc_points)])    
    
#     # TODO : refactor so that formula below follows same pattern used throughout
#     #        convert origin to 2x1 array
#     #        edit code throughout to accept origin as 2D array
#     # origin = origin.reshape(2,1)
#     # arc = origin + radius * np.array([np.cos(arc_points),                                             
#     #                                       np.sin(arc_points)])
   
#    	# PLOT: 
#    	# ARC
#     if set_plot_colour:
#     	#plt.plot(arc[x], arc[y], color=plt.cm.cool(plot_colour))
#     	plt.plot(arc[x], arc[y], c=plot_colour)  
#     else:
#     	plt.plot(arc[x], arc[y])

#     # start point
#     if n == 1: 	
#     	if fixed_at_bottom:
#     		plt.plot(arc[x, 0], arc[y, 0], 'ro')
#     	else:
#     		plt.plot(arc[x, 0], arc[y, 0], 'g^')
#     else:
#     	plt.plot(arc[x, 0], arc[y, 0], 'ko')

#     # origin
#     #plt.plot(origin[x], origin[y], 'go')
#     # plt.xlim(0, 40)
#     # plt.ylim(0, 40)
#     # plt.xlim = (-300, 300)
#     # plt.ylim = (-300, 300)
#     #plt.axis('equal') 
    


#     # define link end angle 
#     end_point = arc[:, -1]   
#     end_angle = angle_to_Xdatum(end_point, 
#                                 origin, radius)
    
#     print(f"end_angle {end_angle} \norigin {origin} \nend_point {end_point} \n \n")
    
#     return end_angle, origin, end_point 
    

    
# # BL = [1, 0, 1, 1, 0 ,0 ,1]
# # BL = [1, 0, 1, 0, 0 , 1 ,0]
    
# # F = actuator_1way_series(4, fixed_at_bottom = True, link_states = BL) 

# # F = actuator_1way_series(4, fixed_at_bottom = False, link_states = BL,
# #                          addtnl_links = [0, 1, 0, 1, 0, 1, 0, 1])
# # plt.show()
 

  
        
        
        
        
        
        
        
        
        

