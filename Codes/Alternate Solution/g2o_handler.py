import gtsam
import numpy as np
import sys



def read_g2o_2D(file_name):
    file = open(file_name, "r")

    poses = []
    edges = []

    for l in file:
        l = l.rstrip().split(' ')
        if l[0] == "VERTEX_SE2":
            p = [float(x) for x in l[1:]]
            poses.append(p)
        elif l[0] == "EDGE_SE2":
            e = [float(x) for x in l[1:]]
            edges.append(e)
    
    
    return np.asarray(poses), np.asarray(edges)

def get_2D_edge_info(edge):
    IDs = [int(edge[0]), int(edge[1])]
    pose = gtsam.Pose2(edge[2],edge[3],edge[4])

    info = np.asarray(np.triu(np.ones(3),0))
    info[info==1] = edge[5:11]
    info = info.transpose()
    info = (info+info.transpose())- np.eye(3)*info

    return IDs, pose, info    

def read_g2o_3D(file_name):
    file = open(file_name, "r")

    poses = []
    edges = []

    for l in file:
        l = l.rstrip().split(' ')
        if l[0] == "VERTEX_SE3:QUAT":
            p = [float(x) for x in l[1:]]
            poses.append(p)
        elif l[0] == "EDGE_SE3:QUAT":
            e = [float(x) for x in l[1:]]
            edges.append(e)
    
    
    return np.asarray(poses), np.asarray(edges)

def get_3D_edge_info(edge):
    IDs = [int(edge[0]), int(edge[1])]
    
    r = gtsam.Rot3.Quaternion(edge[8],edge[5],edge[6],edge[7])
    t = gtsam.Point3(edge[2],edge[3],edge[4])
    pose = gtsam.Pose3(r,t)

    info = np.asarray(np.triu(np.ones(6),0))
    info[info==1] = edge[9:30]
    info = info.transpose()
    info = (info+info.transpose())- np.eye(6)*info

    return IDs, pose, info    