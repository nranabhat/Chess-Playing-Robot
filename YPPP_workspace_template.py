# -*- coding: utf-8 -*-
"""
@author: Peter Adamczyk
"""

# "numpy" for numerical operations
import numpy as np   
# matplotlib -- pyplot for the graphing
from matplotlib import pyplot as plt   
# matplotlib toolkit for 3D plotting. Different graphs are methods of the Axes3D class. 
from mpl_toolkits.mplot3d import Axes3D   

l1 = 0.074
l2 = 0.101
l3 = 0.09837
l4 = 0.133

a01min = np.radians(-90)
a01max = np.radians(90)
na01 = 10 # HOW MANY STEPS - AN INTEGER

b12min = np.radians(-180)
b12max = np.radians(0)
nb12 = 10 # HOW MANY STEPS - AN INTEGER

b23min = np.radians(-60)
b23max = np.radians(160)
nb23 = 10 # HOW MANY STEPS - AN INTEGER

b34min = np.radians(-90)
b34max = np.radians(90)
nb34 = 10 # HOW MANY STEPS - AN INTEGER

# Set up empty arrays to collect the Angles and the Endpoints in n-by-3 arrays. 
# Initialize them with no rows (length 0). 
joint_angles = np.ndarray((0,4)) 
xyz_endpoint = np.ndarray((0,3)) 

# Loop to build the workspace plot
for alpha in np.linspace(a01min,a01max,na01): 
    # Set the Transformation matrix for the "base" link
    T01 = np.array([[np.cos(alpha), (-1)*np.sin(alpha), 0, 0 ],
                    [np.sin(alpha), np.cos(alpha)     , 0, 0 ],
                    [0            , 0                 , 1, l1],
                    [0            , 0                 , 0, 1 ]])
    
    for beta1 in np.linspace(b12min,b12max,nb12): 
        # Set the Transformation matrix for the "upper arm" link
        T12 = np.array([[np.cos(beta1), 0, np.sin(beta1), 0.01],
                        [0            , 1, 0                 , 0],
                        [(-1)*np.sin(beta1), 0, np.cos(beta1)     , 0],
                        [0            , 0, 0                 , 1]])
        
        for beta2 in np.linspace(b23min,b23max,nb23): 
            # Set the Transformation matrix for the "forearm" link
            T23 = np.array([[np.cos(beta2), 0, np.sin(beta2), l2],
                            [0            , 1, 0                 , 0 ],
                            [(-1)*np.sin(beta2), 0, np.cos(beta2)     , 0 ],
                            [0            , 0, 0                 , 1 ]])
            for beta3 in np.linspace(b34min,b34max,nb34): 
            # Set the Transformation matrix for the "forearm" link
                T34 = np.array([[np.cos(beta3), 0, np.sin(beta3), l3],
                            [0            , 1, 0                 , 0 ],
                            [(-1)*np.sin(beta3), 0, np.cos(beta3)     , 0 ],
                            [0            , 0, 0                 , 1 ]])
            
                # Store the joint angles
                joint_angles = np.append(joint_angles, [[alpha, beta1, beta2, beta3]], axis=0)
            
                # Compute the endpoint. 
                # NOTE the Augmented vector to the endpoint in the local "forearm" frame, made using np.array().  
                # NOTE the indexing [0:3] to eliminate the Augmentation and return a 1-by-3 vector.  
                local_vector_aug = np.array([l4,0,0,1])
                # NOTE the use of the method "dot" of the "np.array" object to perform Matrix Multiplication! 
                pt = T01.dot(T12).dot(T23).dot(T34).dot(local_vector_aug)[0:3]
            
                # Accumulate the endpoints in a big array, n-by-3. 
                # Note the use of "np.array()" with the "ndmin=2" argument  
                #  to make the vector above into a 2-D matrix. 
                xyz_endpoint = np.append(xyz_endpoint, np.array(pt, ndmin=2), axis=0)



# Make a nice 3D figure of the Workspace. 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')  # with   projection='3d', this axis will be Axes3D object
ax.scatter(xyz_endpoint[:,0],xyz_endpoint[:,1], marker='o',c=xyz_endpoint[:,2]) #xyz_endpoint[:,2]
ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
#ax.set_aspect('equal')
plt.title('Reachable Workspace of YPPP Robot')
plt.show()

# def get_pts(l1,l2,l3, alpha, beta1, beta2):
#     T01 = np.array([[np.cos(alpha), (-1)*np.sin(alpha), 0, 0 ],
#                     [np.sin(alpha), np.cos(alpha)     , 0, 0 ],
#                     [0            , 0                 , 1, l1],
#                     [0            , 0                 , 0, 1 ]])
    
#     T12 = np.array([[np.cos(beta1), 0, (-1)*np.sin(beta1), 0],
#                     [0            , 1, 0                 , 0],
#                     [np.sin(beta1), 0, np.cos(beta1)     , 0],
#                     [0            , 0, 0                 , 1]])
    
#     T23 = np.array([[np.cos(beta2), 0, (-1)*np.sin(beta2), l2],
#                     [0            , 1, 0                 , 0 ],
#                     [np.sin(beta2), 0, np.cos(beta2)     , 0 ],
#                     [0            , 0, 0                 , 1 ]])
    
#     local_vector_aug = np.array([l3,0,0,1])
#     pts = T01.dot(T12).dot(T23).dot(local_vector_aug)[0:3]
#     return pts



# print("\n \n Parameter = (l1,l2,l3, alpha, beta1, beta2) \n")

# ex_pt1 = get_pts(0.5,1.5,0.8, np.radians(-45), np.radians(-10), np.radians(20))
# #print("Case 1: (1.0,0.55,0.1, 50, 50, 10) --> position: ", ex_pt1,"\n")
# print(ex_pt1)

# ex_pt2 = get_pts(0.5,1.5,0.8, np.radians(-30), np.radians(-30), np.radians(10))
# #print("Case 2: (0,0.15,0.20, 10, 15, 90) --> position: ", ex_pt2,"\n")
# print(ex_pt2)