# Pose-Graph-SLAM


## 1.A 
We first develop a customized function to read 2D Intel dataset from G2O format and output poses and edges. These poses and edges are used in later parts for 2D and 3D SLAM.

Below information summarizes the file format for the 2D SLAM vertices and edges, which is helpful to start with reading and understanding the data.

### Vertices
The vertex information represents the current state of the parameters.

#### 2D Robot Pose
_VERTEX_SE2 i x y theta_

Specifies a 2D robot pose x_i = (x, y, theta)^T where (x,y) represents the translation and theta the rotation.

#### 2D Landmarks / Features
_VERTEX_XY i x y_

Specifies a 2D landmark which is located at p_i = (x, y)^T.

### Edges
The edges encode the measurements obtained by the robot.

#### Odometry or Loop Closures
The odometry of a robot connects subsequent vertices with a relative transformation which specifies how the robot moved according to its measurements. Additionally, non sequential poses can be related to each other by matching their observations.

EDGE_SE2 i j x y theta info(x, y, theta)

Where z_ij = (x, y, theta)^T is the measurement moving from x_i to x_j, i.e. x_j = x_i \oplus z_ij

Basically: \
<img width="851" alt="image" src="https://github.com/DhyeyR-007/Pose-Graph-SLAM/assets/86003669/f3c5d466-5a76-49fa-b5f3-e868ae5bc223">


## 1.B
**Batch Solution:** A batch solution means when we construct the entire graph and solve it at the end altogether. We first load _data/input_INTEL_g2o.g2o_ and construct a 2D nonlinear factor graph using GTSAM, using the Gauss-Newton solver.

## 1.C
**Incremental Solution:** Here, we use ISAM2 solver to optimize the trajectory incrementally i.e. as we build the graph gradually.


## 2.A
 Write a function to read 3D Garage G2O file 4 from G2O format and output poses and edges
