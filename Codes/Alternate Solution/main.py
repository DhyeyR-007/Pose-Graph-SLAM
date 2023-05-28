from g2o_handler import *
import matplotlib.pyplot as plt
import math
from mpl_toolkits import mplot3d
def Vector3(x, y, z): return np.array([x, y, z])
def Vector6(x, y, z, r, j, k): return np.array([x, y, z, r, j, k])

def gn_2d(input_file, output_file):
    poses, edges = read_g2o_2D(input_file)

    graph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()

    for i in range(poses.shape[0]):
        init.insert(int(poses[i,0]),gtsam.Pose2(poses[i,1],poses[i,2],poses[i,3]))

    prior_model = gtsam.noiseModel.Diagonal.Variances(Vector3(0.5, 0.5, 0.1))
    graph.add(gtsam.PriorFactorPose2(0, init.atPose2(0), prior_model))

    for j in range(edges.shape[0]):
        edge = edges[j]
        IDs, pose, info = get_2D_edge_info(edge)
        cov = np.linalg.inv(info)
        model = gtsam.noiseModel.Gaussian.Covariance(cov)
        graph.add(gtsam.BetweenFactorPose2(IDs[0],IDs[1],pose,model))


    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(graph, init, params)
    result = optimizer.optimize()

    gtsam.writeG2o(graph,result,output_file)

    opt_poses = gtsam.utilities.extractPose2(result)
    
    plt.plot(poses[:,1],poses[:,2])
    plt.plot(opt_poses[:,0],opt_poses[:,1])
    plt.title('Gauss-Netwon Batch 2D')
    plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
    plt.axis('equal')
    plt.show()
    
def isam_2d(input_file,output_file):
    poses, edges = read_g2o_2D(input_file)

    graph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()

    params = gtsam.ISAM2Params()
    isam = gtsam.ISAM2(params)

    for i in range(poses.shape[0]):
        
        if(i==0):
            init.insert(int(poses[i,0]),gtsam.Pose2(poses[i,1],poses[i,2],poses[i,3]))
            prior_model = gtsam.noiseModel.Diagonal.Variances(Vector3(0.5, 0.5, 0.1))
            graph.add(gtsam.PriorFactorPose2(0, init.atPose2(0), prior_model))
        else:
            init.insert(int(poses[i,0]),result.atPose2(i-1))

        for j in range(edges.shape[0]):
            edge = edges[j]
            if(edge[1]==i):
                IDs, pose, info = get_2D_edge_info(edge)
                cov = np.linalg.inv(info)
                model = gtsam.noiseModel.Gaussian.Covariance(cov)
                graph.add(gtsam.BetweenFactorPose2(IDs[0],IDs[1],pose,model))

        isam.update(graph,init)
        result = isam.calculateEstimate()
        graph.resize(0)
        init.clear()
        

    gtsam.writeG2o(graph,result,output_file)

    opt_poses = gtsam.utilities.extractPose2(result)

    plt.plot(poses[:,1],poses[:,2])
    plt.plot(opt_poses[:,0],opt_poses[:,1])
    plt.title('isam2 2D')
    plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
    plt.axis('equal')
    plt.show()

def gn_3d(input_file, output_file):
    poses, edges = read_g2o_3D(input_file)

    graph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()

    for i in range(poses.shape[0]):
        r = gtsam.Rot3.Quaternion(poses[i,7],poses[i,4],poses[i,5],poses[i,6])
        t = gtsam.Point3(poses[i,1],poses[i,2],poses[i,3])
        init.insert(int(poses[i,0]),gtsam.Pose3(r,t))
        
    
    prior_model = gtsam.noiseModel.Diagonal.Variances(Vector6(0.5, 0.5, 0.5,0.1,0.1, 0.1))
    graph.add(gtsam.PriorFactorPose3(0, init.atPose3(0), prior_model))

    for j in range(edges.shape[0]):
        edge = edges[j]
        IDs, pose, info = get_3D_edge_info(edge)
        cov = np.linalg.inv(info)
        model = gtsam.noiseModel.Gaussian.Covariance(cov)
        graph.add(gtsam.BetweenFactorPose3(IDs[0],IDs[1],pose,model))


    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(graph, init, params)
    result = optimizer.optimize()

    gtsam.writeG2o(graph,result,output_file)

    opt_poses = gtsam.utilities.extractPose3(result)

    ax = plt.axes(projection='3d')
    plt.plot(poses[:,1],poses[:,2],poses[:,3])
    plt.plot(opt_poses[:,9],opt_poses[:,10],opt_poses[:,11])
    plt.title('Gauss-Netwon Batch 3D')
    plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
    
    X = opt_poses[:,9]
    Y = opt_poses[:,10]
    Z = opt_poses[:,11]
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0

    mid_x = (X.max()+X.min()) * 0.5
    mid_y = (Y.max()+Y.min()) * 0.5
    mid_z = (Z.max()+Z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()


def isam_3d(input_file,output_file):
    poses, edges = read_g2o_3D(input_file)

    graph = gtsam.NonlinearFactorGraph()
    init = gtsam.Values()

    params = gtsam.ISAM2Params()
    isam = gtsam.ISAM2(params)

    for i in range(poses.shape[0]):
        
        if(i==0):
            r = gtsam.Rot3.Quaternion(poses[i,7],poses[i,4],poses[i,5],poses[i,6])
            t = gtsam.Point3(poses[i,1],poses[i,2],poses[i,3])
            init.insert(int(poses[i,0]),gtsam.Pose3(r,t))
            prior_model = gtsam.noiseModel.Diagonal.Variances(Vector6(0.5, 0.5, 0.5,0.1,0.1, 0.1))
            graph.add(gtsam.PriorFactorPose3(0, init.atPose3(0), prior_model))
        else:
            init.insert(int(poses[i,0]),result.atPose3(i-1))

        for j in range(edges.shape[0]):
            edge = edges[j]
            if(edge[1]==i):
                IDs, pose, info = get_3D_edge_info(edge)
                cov = np.linalg.inv(info)
                model = gtsam.noiseModel.Gaussian.Covariance(cov)
                graph.add(gtsam.BetweenFactorPose3(IDs[0],IDs[1],pose,model))

        isam.update(graph,init)
        result = isam.calculateEstimate()
        graph.resize(0)
        init.clear()
        

    gtsam.writeG2o(graph,result,output_file)

    opt_poses = gtsam.utilities.extractPose3(result)

    ax = plt.axes(projection='3d')
    plt.plot(poses[:,1],poses[:,2],poses[:,3])
    plt.plot(opt_poses[:,9],opt_poses[:,10],opt_poses[:,11])
    plt.title('isam2 3D')
    plt.legend(['Unoptimized Trajectory', 'Optimized Trajectory'])
    
    X = opt_poses[:,9]
    Y = opt_poses[:,10]
    Z = opt_poses[:,11]
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0

    mid_x = (X.max()+X.min()) * 0.5
    mid_y = (Y.max()+Y.min()) * 0.5
    mid_z = (Z.max()+Z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

def main():
    question_num = sys.argv[1]
    input_file = sys.argv[2]
    output_file = sys.argv[3]

    if(question_num=='1'):
        gn_2d(input_file,output_file)
    elif(question_num=='2'):
        isam_2d(input_file,output_file)
    elif(question_num=='3'):
        gn_3d(input_file,output_file)
    elif(question_num=='4'):
        isam_3d(input_file,output_file)
    
if __name__ == "__main__":
    main()