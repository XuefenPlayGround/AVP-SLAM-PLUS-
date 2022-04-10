#include <iostream>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>
#include <queue>
#include <cmath>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "parser");

    std::string fileName;
    double tooFar = 0.01;
    int ignoredClouds = 300;
    std::string dataFile = "/home/catkin_ws/src/AVP-SLAM-PLUS/parse_rosbag/data/";
    double odomCovXX = 1;
    double odomCovYY = 1;
    double odomCovTT = 1;
    double maxCovXX = 1;
    double maxCovYY = 1;
    double maxCovTT = 1;

    ros::NodeHandle nh;
   
    // get parameter from config file
    nh.param<std::string>("dataFile", dataFile, "/home/catkin_ws/src/AVP-SLAM-PLUS/parse_rosbag/data/");
    nh.param<std::string>("fileName", fileName, "");
    nh.param<double>("tooFar", tooFar, 0.01);
    nh.param<int>("ignoredClouds",ignoredClouds,300);
    nh.param<double>("odomCovXX",odomCovXX,1);
    nh.param<double>("odomCovYY",odomCovYY,1);
    nh.param<double>("odomCovTT",odomCovTT,1);
    nh.param<double>("maxCovXX",maxCovXX,1);
    nh.param<double>("maxCovYY",maxCovYY,1);
    nh.param<double>("maxCovTT",maxCovTT,1);
    
    rosbag::Bag bag;
    std::string dataDir = dataFile+"rosbag/";
    std::string outDir = dataFile+"g2o/";
    std::string GTDir = dataFile+"GroundTruth/";
    std::string OdomDir = dataFile+"odometry/";

    bag.open(dataDir+fileName+".bag", rosbag::bagmode::Read);

    // file to save the vertex and edges to dedicated g2o file
    std::ofstream myfile;
    std::ofstream GTfile;
    std::ofstream Odomfile;
    myfile.open(outDir+fileName+".g2o");
    GTfile.open(GTDir+fileName+".g2o");
    Odomfile.open(OdomDir+fileName+".g2o");

    Eigen::Affine3f transform_0 = Eigen::Affine3f::Identity();
    int initialized = 0;
    Eigen::Affine3f transform_t = Eigen::Affine3f::Identity();

    std::vector<double> slamX;
    std::vector<double> slamY;
    std::vector<double> slamTheta;

    int vertexCount = 0;
    bool GT_first = false; //store GT before SLAM pose so the id would start at 0 

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        std::string topic = m.getTopic();
        //ros::Time time = m.getTime();
        //std::cout<<"topic"<<topic<<std::endl;
        //std::cout<<"Time "<<time<<std::endl;

        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL){
            std::string text = s->data;
            // myfile << text << std::endl;
            
            size_t pos = text.find(" ");
            std::string name = text.substr(0, pos);
            if(name == "VERTEX_SE2"){
                text = text.erase(0,pos+1);
                pos = text.find(" ");
                //ignore time
                text = text.erase(0,pos+1);

                //trim x
                pos = text.find(" ");
                double currentX = std::stod(text.substr(0,pos));
                text = text.erase(0,pos+1);
                //trim y
                pos = text.find(" ");
                double currentY = std::stod(text.substr(0,pos));
                text = text.erase(0,pos+1);
                //save yaw
                double currentYaw = std::stod(text);

                if(initialized==0){
                    transform_0.translation() << currentX, currentY, 0.0;
                    transform_0.rotate (Eigen::AngleAxisf (currentYaw, Eigen::Vector3f::UnitZ()));
                    initialized = 1;
                }
                else{
                    transform_t = Eigen::Affine3f::Identity();
                    transform_t.translation() << currentX, currentY, 0.0;
                    transform_t.rotate (Eigen::AngleAxisf (currentYaw, Eigen::Vector3f::UnitZ()));
                }
                Odomfile << "VERTEX_SE2 " << vertexCount << " " << currentX << " " << currentY << " " << currentYaw << std::endl;
                
            }
            
        }

        nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
        if (odom != nullptr && topic=="/currentPose" && GT_first == true){
        //     nav_msgs::Odometry odom_msg = odom->data;
            
            double roll, pitch, yaw;
            //Transform quaternion to rotation
            tf::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            myfile << "VERTEX_SE2 " << vertexCount << " " << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << yaw << std::endl;
            slamX.push_back(odom->pose.pose.position.x);
            slamY.push_back(odom->pose.pose.position.y);
            slamTheta.push_back(yaw);

            if(vertexCount>0){
                Eigen::Affine3f transform_0t = transform_0.inverse()*transform_t;
                transform_0 = transform_t;
                double theta = atan2(-transform_0t(0, 1), transform_0t(0, 0));
                myfile << "EDGE_SE2 " << vertexCount-1 << " " << vertexCount << " " << transform_0t(0, 3) << " " << transform_0t(1, 3) << " " << theta << " " << odomCovXX << " 0 0 " << odomCovYY << " 0 " << odomCovTT << std::endl;
            }
            
            vertexCount++;
        }

        else if (odom != nullptr && topic=="/odom"){
        //     nav_msgs::Odometry odom_msg = odom->data;
            
            double roll, pitch, yaw;
            //Transform quaternion to rotation
            tf::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            GTfile << "VERTEX_SE2 " << vertexCount << " " << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << yaw << std::endl;
            GT_first = true;
        }
    }

    bag.close();
    std::cout<<"done reading "<<slamX.size()<<" odometry vertexes"<<std::endl;

    int newEdgeCount = 0;
    for(size_t j=ignoredClouds;j<slamX.size();j++){
        int closestMatch = -1;
        double closestDist = 1000;
        for(size_t i=0;i<j-ignoredClouds;i++){
            double dist = pow((slamX[i]-slamX[j]),2)+pow((slamY[i]-slamY[j]),2);
            if(dist<pow(tooFar,2)){
                if(dist<closestDist){
                    closestMatch=i;
                    closestDist = dist;
                }
            }
        }
        if(closestMatch!=-1){
            size_t i = closestMatch;
            double covXX = maxCovXX*abs(slamX[i]-slamX[j])/tooFar;
            double covYY = maxCovYY*abs(slamY[i]-slamY[j])/tooFar;
            myfile << "EDGE_SE2 " << i << " " << j << " " << 0 << " " << 0 << " " << slamTheta[j]-slamTheta[i] << " "<<covXX<<" 0 0 "<<covYY<<" 0 "<<maxCovTT<< std::endl;
            //std::cout<<"i "<<i<<" "<<slamX[i]<<","<<slamY[i]<<" j "<<j<<" "<<slamX[j]<<","<<slamY[j]<<std::endl;
            
            newEdgeCount++;
        }
    }



    myfile.close();
    GTfile.close();
    Odomfile.close();
    
    std::cout << "added " << newEdgeCount << " edges" << std::endl;

    return 0;
}


