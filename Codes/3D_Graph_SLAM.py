### Author: Dhyey Manish Rajani ###

import gtsam
from gtsam.utils import plot
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv




#### Q2.A. ####


# function to read g20 file manually
def read(filename, title1, title2):

    vertex = []  # if line starts with 'VERTEX_SE3:QUAT' we append here
    edge = []  # if line starts with 'EDGE_SE3:QUAT' we append here
    with open(filename) as file:
        for line in file:
            line = line.split()
            
            # if line starts with 'VERTEX_SE3:QUAT'....
            # [VERTEX_SE3:QUAT i x y z qx qy qz qw] ~ [line[0] line[1]  line[2:]]
            if line[0] == title1:
                v = int(line[1])
                pose = np.array(line[2:], dtype = np.float32)  # from x to qw in a single array called pose
                
                # combining v and pose into vertex
                vertex.append([v, pose])

            # if line starts with 'EDGE_SE3:QUAT' ....
            # [EDGE_SE3:QUAT i j x y z qx qy qz qw info(x, y, z, qx, qy, qz)] ~ [line[0] line[1]  line[2:]  line[3:10] line[10:]] 
            elif line[0] == title2:

                pose = np.array(line[3:10], dtype= np.float32) # from x to qw
                info = np.array(line[10:], dtype=np.float32)  # info(x, y, z, qx, qy, qz)

                # combining line[1] = i; line[2] = j ; pose and info into edge
                edge.append([int(line[1]), int(line[2]), pose, info])


    return vertex, edge



(vertex, edge) = read('parking-garage.g2o', title1 = 'VERTEX_SE3:QUAT', title2 = 'EDGE_SE3:QUAT' )


# --------------------------------------------------------------------------------------------------------- #

#### Q2.B. {BATCH SOLUTION} ####

# load an initial graph and reconstructing it(using direct function here)
graph, initial = gtsam.readG2o('parking-garage.g2o', is3D = True)

# Defining Prior on the pose having index (key) = 0
priorModel = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))

# adding the prior factor to avoid gauge problem 
graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(), priorModel))

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
resultPoses = gtsam.utilities.extractPose3(result)
InitialPoses = gtsam.utilities.extractPose3(initial)



# Plotting the initial and results as Unoptimized and Optimized Trajectory respectively
ax = plt.axes(projection='3d')
ax.plot3D(InitialPoses[:,9], InitialPoses[:,10], InitialPoses[:,11], label="Unoptimized Trajectory")
ax.plot3D(resultPoses[:,9], resultPoses[:,10], resultPoses[:,11], label="Optimized Trajectory")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.legend(loc='upper right')
plt.title('Q2.B.')
plt.show()

# -------------------------------------------------------------------------------------------------------------------------------- #


#### Q2.C. {INCREMENTAL SOLUTION} ####


# Function for converting 1 × 21 info vector ------>  6x6 information matrix  ---inverse--->   to covariance matrix.
#                                                                                                             
#            
def constructCov(info):
        A = np.array([[info[0], info[1], info[2], info[3], info[4], info[5]],

                      [info[1], info[6], info[7], info[8], info[9], info[10]],

                      [info[2], info[7], info[11], info[12], info[13], info[14]],

                      [info[3], info[8], info[12], info[15], info[16], info[17]],

                      [info[4], info[9], info[13], info[16], info[18], info[19]],

                      [info[5], info[10], info[14], info[17], info[19], info[20]]])
        
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
        (idp, arr) = pose

        x = arr[0]
        y = arr[1]
        z = arr[2]        
        qx = arr[3]
        qy = arr[4]
        qz = arr[5]
        qw = arr[6]
        

        # for plotting unoptimized trajectory
        init_list.append([x , y , z])

        # for the first pose
        if idp == 0:

            # Noise model definition
            priorNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-4, 1e-4, 1e-4, 1e-8, 1e-8, 1e-8], dtype=float))

            
            # Rotation
            R1 = gtsam.Rot3.Quaternion(qw,qx,qy,qz)
            
            # Translation
            T1 = gtsam.Point3(x,y,z)

            # adding prior pose to the graph
            graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(R1, T1), priorNoise)) 

            # storing the initial data for initial estimate
            initial_estimate.insert(idp, gtsam.Pose3(R1,T1))
        
            
        
        # if not the first pose
        else:
            # last optimized pose
            prevPose = result.atPose3(idp-1)

            # storing the initial and/or current data for initial estimate
            initial_estimate.insert(idp, prevPose)

            for everyedge in edge:

                # Extract information from the current edge
                (ide1,ide2,pose,info) = everyedge

                # if 'to' pose index  ==  pose index
                if ide2 == idp:

                    # extracting x,y,z,qx,qy,qz,qw for edges
                    dx = pose[0]
                    dy = pose[1]
                    dz = pose[2]

                    dqx = pose[3]
                    dqy = pose[4]
                    dqz = pose[5]
                    dqw = pose[6]

                    # Covariance Matrix Construction from information vector
                    cov = constructCov(info)
                    
                    Model = gtsam.noiseModel.Gaussian.Covariance(cov)

                    R = gtsam.Rot3.Quaternion(dqw,dqx,dqy,dqz)
                    T = gtsam.Point3(dx,dy,dz)

                    # adding edge to graph
                    graph.add(gtsam.BetweenFactorPose3(ide1, ide2, gtsam.Pose3(R,T), Model))
        
       
        isam.update(graph, initial_estimate)        # updating isam solver
        result = isam.calculateEstimate()           # estimating results




# Extracting poses for plotting
resultPoses = gtsam.utilities.extractPose3(result)
InitialPoses = np.asarray(init_list)



# Plotting the initial and results as Unoptimized and Optimized Trajectory respectively
ax = plt.axes(projection='3d')
ax.plot3D(InitialPoses[:,0], InitialPoses[:,1], InitialPoses[:,2], label="Unoptimized Trajectory")
ax.plot3D(resultPoses[:,9], resultPoses[:,10], resultPoses[:,11], label="Optimized Trajectory")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.title('Q2.C.')
plt.legend(loc='upper right')
plt.show()