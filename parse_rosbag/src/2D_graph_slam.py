#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import yaml
from gtsam import NonlinearFactorGraph,PriorFactorPose2,noiseModel_Gaussian,noiseModel_Diagonal,Values,Pose2,GaussNewtonParams,GaussNewtonOptimizer,BetweenFactorPose2,ISAM2,ISAM2Params

def readSE2(file):
    vertex_SE2_list = []
    edge_SE2_list = []
    
    
    with open(file) as f:
        lines = f.readlines()
        for line in lines:
            line = line.split(' ')
            if(line[0]=='VERTEX_SE2'):
                i = int(line[1])

                x = float(line[2])
                y = float(line[3])
                theta = float(line[4])

                vertex = (i,Pose2(x,y,theta))
                vertex_SE2_list.append(vertex)
            elif(line[0]=='EDGE_SE2'):
                i = int(line[1])
                j = int(line[2])

                x = float(line[3])
                y = float(line[4])
                theta = float(line[5])
                
                info = [float(x) for x in line[6:]]
                omega = np.zeros((3,3))
                for row in range(3):
                    for col in range(3):
                        if(col>=row):
                            omega[row,col] = info[0]
                            info = info[1:]
                        else:
                            omega[row,col] = omega[col,row]
                #model = NoiseModelFactor.Gaussian.Information(omega)
                model = noiseModel_Gaussian.Covariance(omega)
                
                # sigma = np.linalg.inv(omega)
                # model = NoiseModelFactor.Gaussian.Covariance(sigma)

                edge = BetweenFactorPose2(i, j, Pose2(x,y,theta), model)
                edge_SE2_list.append(edge)
            else:
                print('error',line)

    return vertex_SE2_list, edge_SE2_list

def batchSolution(output,GT,Odom):
    vertexes = output[0]
    edges = output[1]

    graph = NonlinearFactorGraph()
    # Add a prior on the first pose, setting it to the origin
    priorNoise = noiseModel_Gaussian.Covariance(initialCov)
    graph.add(PriorFactorPose2(0, Pose2(initialPose), priorNoise))

    # Add odometry factors
    for edge in edges:
        graph.add(edge)

    # Create the data structure to hold the initialEstimate estimate to the solution
    initialEstimate = Values()
    for vertex in vertexes:
        initialEstimate.insert(*vertex)

    # Optimize the initial values using a Gauss-Newton nonlinear optimizer
    parameters = GaussNewtonParams()
    parameters.setRelativeErrorTol(relativeErrorTol)
    parameters.setMaxIterations(maxIterations)
    optimizer = GaussNewtonOptimizer(graph, initialEstimate, parameters)
    result = optimizer.optimize()
    #result.print("Final Result:\n")
    
    #store resulting poses
    resultPoses = []
    for key in range(result.keys().size()):
        resultPoses.append((result.keys().at(key),result.atPose2(key)))

    #plot results
    initialX = np.array([pose[1].x() for pose in vertexes])
    initialY = np.array([pose[1].y() for pose in vertexes])
    resultX = np.array([pose[1].x() for pose in resultPoses])
    resultY = np.array([pose[1].y() for pose in resultPoses])
    GTX = np.array([pose[1].x() for pose in GT[0]])
    GTY = np.array([pose[1].y() for pose in GT[0]])
    OdomX = np.array([pose[1].x() for pose in Odom[0]])
    OdomY = np.array([pose[1].y() for pose in Odom[0]])
    
    plt.figure()
    plt.plot(initialX,initialY,"r",label='SLAM')
    plt.plot(resultX,resultY,"g",label='Loop Closure')
    plt.plot(GTX,GTY,"b",label='GT')
    plt.plot(OdomX,OdomY,"black",label='Odometry')
    plt.title('Factor Graph batch')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.draw()

    #calculate erorr TODO

def incrementalSolution(output,GT,Odom):
    vertexes = output[0]
    edges = output[1]

    parameters = ISAM2Params()
    parameters.setRelinearizeThreshold(relinearizeThreshold)
    parameters.setRelinearizeSkip(relinearizeSkip)
    isam = ISAM2(parameters)
    isam.update()
    isam.calculateEstimate()

    # Loop over the poses, adding the observations to iSAM incrementally
    for vertex in vertexes:
        graph = NonlinearFactorGraph()
        initialEstimate = Values()
        
        startIdx = 0
        if(vertex[0] == startIdx):
            priorNoise = noiseModel_Gaussian.Covariance(initialCov)
            graph.add(PriorFactorPose2(0, Pose2(initialPose), priorNoise))
            initialEstimate.insert(*vertex)
        else:
            prevPose = result.atPose2(vertex[0]-1)
            initialEstimate.insert(vertex[0],prevPose)
            for edge in edges:
                if(edge.keys().at(1)==vertex[0]):
                    graph.add(edge)

        isam.update(graph,initialEstimate)
        result = isam.calculateEstimate()

    #store resulting poses
    resultPoses = []
    for key in range(result.keys().size()):
        resultPoses.append((result.keys().at(key),result.atPose2(key)))

    #plot results
    initialX = [pose[1].x() for pose in vertexes]
    initialY = [pose[1].y() for pose in vertexes]
    resultX = [pose[1].x() for pose in resultPoses]
    resultY = [pose[1].y() for pose in resultPoses]
    GTX = [pose[1].x() for pose in GT[0]]
    GTY = [pose[1].y() for pose in GT[0]]
    OdomX = [pose[1].x() for pose in Odom[0]]
    OdomY = [pose[1].y() for pose in Odom[0]]

    plt.figure()
    plt.plot(initialX,initialY,"r",label='SLAM')
    plt.plot(resultX,resultY,"g",label='Loop Closure')
    plt.plot(GTX,GTY,"b",label='GT')
    plt.plot(OdomX,OdomY,"black",label='Odometry')
    plt.title('ISAM2 incremental')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.draw()

if __name__ == "__main__":
    #part A
    with open('/home/jacklee/catkin_ws/src/AVP-SLAM-PLUS/parse_rosbag/config/configFile.yaml','r') as stream:
        try:
            config = yaml.safe_load(stream)
            dataDir = config['dataFile']+'g2o/'
            GTDir = config['dataFile']+'GroundTruth/'
            OdomDir = config['dataFile']+'odometry/'
            fileName = config['fileName']
            relativeErrorTol = config['relativeErrorTol']
            maxIterations = config['maxIterations']
            relinearizeThreshold = config['relinearizeThreshold']
            relinearizeSkip = config['relinearizeSkip']
            initialPose = np.asarray(config['initialPose']).reshape(3,)
            initialCov = np.asarray(config['initialCov']).reshape(3,3)
        except yaml.YAMLError as exc:
            print(exc)

    output = readSE2(dataDir+fileName+'.g2o')
    print('read',len(output[0]),'Vertexs and',len(output[1]),'edges')

    GT = readSE2(GTDir+fileName+'.g2o')
    Odom = readSE2(OdomDir+fileName+'.g2o')

    #part B
    batchSolution(output,GT,Odom)
    
    #part C
    incrementalSolution(output,GT,Odom)

    plt.show()
    
    
    
    
    
    
    
    
    
    
    
