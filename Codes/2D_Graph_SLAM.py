### Author: Dhyey Manish Rajani ###

import gtsam
from gtsam.utils import plot
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv


#### Q1.A. ####

# function to read g20 file manually
def read_G2o(fileName, title1, title2):
    
    f = open(fileName, 'r')
    A = f.readlines()
    f.close()


    vertex = []
 
    edge = []


    # iterating over all the lines in the g20 file
    for line in A:
        # if line starts with "VERTEX_SE2"....
        if title1 in line:
            (ver, ind, x, y, theta) = line.split(' ')

            # list named'lisk' is initialized so that an entire line of data can be read in one single list.
            lisk=[]
            lisk.append(int(ind))
            lisk.append(float(x))
            lisk.append(float(y))
            lisk.append(float(theta.rstrip('\n')))

            # Now this list named'lisk' is appended into a bigger list called 'vertex'.
            vertex.append(lisk)


        # if line starts with "EDGE_SE2"....
        if title2 in line:
            (edg, i, j, x1, y1, theta1, q11, q12, q13, q22, q23, q33) = line.split(' ')

            # list named'lis' is initialized so that an entire line of data can be read in one single list.
            lis = []
            lis.append(int(i))
            lis.append(int(j))
            lis.append(float(x1))
            lis.append(float(y1)) 
            lis.append(float(theta1))
            lis.append([float(i.rstrip('\n')) if i==q33 else float(i) for i in [q11,q12,q13,q22,q23,q33] ])
            
            # Now this list named'lis' is appended into a bigger list called 'edge'.
            edge.append(lis)



    return (vertex, edge)



# driver function for reading g20 file
(vertex, edge) = read_G2o('input_INTEL_g2o.g2o', title1 = "VERTEX_SE2", title2 = "EDGE_SE2"  )

# --------------------------------------------------------------------------------------------------------- #

#### Q1.B. {BATCH SOLUTION} ####

# load an initial graph and reconstructing it(using direct function here)
graph, initial = gtsam.readG2o('input_INTEL_g2o.g2o', is3D = False)

# Defining Prior on the pose having index (key) = 0
priorModel = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-6, 1e-6, 1e-8))

# adding the prior factor to avoid gauge problem 
graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), priorModel))

# Defining params for insering in the optimizer
params = gtsam.GaussNewtonParams()   ## the paramters need to be initialized but since 
                                     ## nothing is mentioned in the question we use the predeined parameters in the python wrapper

                                    ## Example of params for regulating optimizer
                                    # params.setRelativeErrorTol(-1e+20)
                                    # params.setAbsoluteErrorTol(-1e+20)
                                    # params.setMaxIterations(10)

# Defining Optimizer
optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)

# Optimizing
result = optimizer.optimize()
print("Optimization complete")

# Extracting poses for plotting
resultPoses = gtsam.utilities.extractPose2(result)
InitialPoses = gtsam.utilities.extractPose2(initial)

# Plotting the initial and results as Unoptimized and Optimized Trajectory respectively
fig, ax = plt.subplots()
ax.plot(InitialPoses[:,0], InitialPoses[:,1], linewidth=1.0, label="Unoptimized Trajectory")
ax.plot(resultPoses[:,0], resultPoses[:,1], linewidth=1.0, label = "Optimized Trajectory")
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.title('Q1.B.')
plt.legend(loc='upper left')
plt.show()

# --------------------------------------------------------------------------------------------------------- #


#### Q1.C. {INCREMENTAL SOLUTION} ####


# Function for converting info vector [q_11, q_12, q_13, q_22 q_23 q_33 ]  ------>  3x3 information matrix Ω =[q_11  q_12  q_13,  ---inverse--->    to covariance matrix.
#                                                                                                              q_12  q_22  q_23,
#                                                                                                              q_13  q_23  q_33 ]
def constructCov(info):
        A = np.array([[info[0], info[1], info[2]],
                      [info[1], info[3], info[4]],
                      [info[2], info[4], info[5]]])
        B = inv(A)  # for converting to covariance matrix
        return B


#for collecting data for unoptimized trajectory
init_list = []  

# Initialize isam solver
isam = gtsam.ISAM2() 

# iterating over each pose in poses....
for pose in vertex:
    
        # Initialize factor graph
        graph = gtsam.NonlinearFactorGraph()
        
        # Initialize initial estimation
        initial_estimate = gtsam.Values()

        # Extract information from the current pose
        (idp, x, y, ang)=pose

        # for plotting unoptimized trajectory
        init_list.append([x , y, ang])

        # for the first pose
        if idp == 0:
            # Noise model definition
            priorNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3,0.3, 5*np.pi/180]))
            
            # adding prior pose to the graph
            graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(x, y, ang), priorNoise))
            
            # storing the initial data for initial estimate
            initial_estimate.insert(idp, gtsam.Pose2(x, y, ang))
        
        # if not the first pose
        else:
            # last optimized pose
            prevPose = result.atPose2(idp-1)

            # storing the initial and/or current data for initial estimate
            initial_estimate.insert(idp, prevPose)

            for everyedge in edge:

                # Extract information from the current edge
                ide1,ide2,dx,dy,d_ang,info = everyedge

                # if 'to' pose index  ==  pose index
                if ide2 == idp:

                    # Covariance Matrix Construction from Information vector
                    cov = constructCov(info) 

                    Model = gtsam.noiseModel.Gaussian.Covariance(cov)
                    
                    # adding edge to graph
                    graph.add(gtsam.BetweenFactorPose2(ide1, ide2, gtsam.Pose2(dx,dy,d_ang), Model))
        
        isam.update(graph, initial_estimate)   # updating isam solver
        result = isam.calculateEstimate()      # estimating results

        

# Extracting poses for plotting
resultPoses = gtsam.utilities.extractPose2(result)
InitialPoses = np.asarray(init_list)



# Plotting the initial and results as Unoptimized and Optimized Trajectory respectively
fig, ax = plt.subplots()
ax.plot(InitialPoses[:,0], InitialPoses[:,1], linewidth=1.0, label="Unoptimized Trajectory")
ax.plot(resultPoses[:,0], resultPoses[:,1], linewidth=1.0, label = "Optimized Trajectory")
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.ylim(-42.5, 10)
plt.xlim(-25, 35)
plt.title('Q1.C.')
plt.legend(loc='upper right')
plt.show()