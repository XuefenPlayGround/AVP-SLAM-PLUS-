#include <iostream>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv/cv.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
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

typedef pcl::PointXYZRGB PointType;

int main(int argc, char *argv[]){
    // ros::init(argc, argv, "bag_it");
    // ros::NodeHandle nh;

    std::string fileName;

    if(argc>=2){
        fileName = argv[1];
    }
    else{
        std::cout<<"argument missing: file name"<<std::endl;
        return 1;
    }

    rosbag::Bag bag;
    std::string dataFile = "/home/catkin_ws/src/AVP-SLAM-PLUS/parse_rosbag/";
    bag.open(dataFile+fileName+".bag", rosbag::bagmode::Read);

    // file to save the vertex and edges to
    ofstream myfile;
    myfile.open (dataFile+"output.g2o");

    Eigen::Affine3f transform_0 = Eigen::Affine3f::Identity();
    int initialized = 0;
    Eigen::Affine3f transform_t = Eigen::Affine3f::Identity();

    std::vector<int> slamID;
    std::vector<double> slamX;
    std::vector<double> slamY;
    std::vector<double> slamTheta;

    int vertexCount = 0;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        std::string topic = m.getTopic();
        ros::Time time = m.getTime();
        std::cout<<"topic"<<topic<<std::endl;
        std::cout<<"Time "<<time<<std::endl;

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

                slamX.push_back(currentX);
                slamY.push_back(currentY);
                slamTheta.push_back(currentYaw);
                slamID.push_back(vertexCount);

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
                
            }
            
        }

        nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
        if (odom != nullptr && topic=="/currentPose"){
        //     nav_msgs::Odometry odom_msg = odom->data;
            
            double roll, pitch, yaw;
            //Transform quaternion to rotation
            tf::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            myfile << "VERTEX_SE2 " << vertexCount << " " << odom->pose.pose.position.x << " " << odom->pose.pose.position.y << " " << yaw << std::endl;

            if(vertexCount>0){
                Eigen::Affine3f transform_0t = transform_0.inverse()*transform_t;
                transform_0 = transform_t;
                double theta = atan2(-transform_0t(0, 1), transform_0t(0, 0));
                myfile << "EDGE_SE2 " << vertexCount-1 << " " << vertexCount << " " << transform_0t(0, 3) << " " << transform_0t(1, 3) << " " << theta << " 0.1 0 0 0.1 0 0.3" << std::endl;
            }
            
            vertexCount++;
        }
    }

    bag.close();
    std::cout<<"done reading "<<slamID.size()<<" odometry vertexes"<<std::endl;

    int newEdgeCount = 0;
    int ignoredClouds = 100;
    for(int j=ignoredClouds;j<slamID.size();j++){
        for(int i=0;i<j-ignoredClouds;i++){
            double tooFar = 0.001;
            if(pow((slamX[i]-slamX[j]),2)+pow((slamY[i]-slamY[j]),2)<pow(tooFar,2)){
                myfile << "EDGE_SE2 " << slamID[i] << " " << slamID[j] << " " << 0 << " " << 0 << " " << 0 << " 10000 1000 1000 10000 1000 10000" << std::endl;
                newEdgeCount++;
            }
        }
    }



    myfile.close();

    std::cout << "added " << newEdgeCount << " edges" << std::endl;

    return 0;
}


