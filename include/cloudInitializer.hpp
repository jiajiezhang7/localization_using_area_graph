#pragma once
#ifndef _CLOUD_INITIALIZER_HPP_
#define _CLOUD_INITIALIZER_HPP_
#include "utility.h"
#include "cloudBase.hpp"


class CloudInitializer : public CloudBase
{
public:
    Eigen::Matrix4f MaxRobotPose;
    double MaxScore;
    // vector to record how many times the ray goes through map to determinate if the points is inside or outside of map
    std::vector<int> numofIntersection;
    std::vector<double> inRayDis;
    std::vector<double> inRayRange;
    //if this point is matched with outside of this area, then it is 1, else 0
    std::vector<double> match_with_outside;
    int numofInsidePoints;
    int numofOutsidePoints;
    double turkeyScore;
    bool bGuessReady;
    ofstream rescueRoomStream;

    ros::Subscriber subInitialGuess;
    ros::Publisher pubRobotGuessMarker;
    ros::Publisher pubRobotPoseAfterICP;
    ros::Publisher pubCurrentMaxRobotPose;;

    geometry_msgs::PointStamped robotGuess;

    int rescueTimes;
    double rescueRunTime;
    
    // image_transport::ImageTransport it;
    // image_transport::Publisher pubThings2Say;

    // void registerNodeHandle(ros::NodeHandle& _nh);
    void setLaserCloudin(pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing_,std_msgs::Header mapHeader_);
    void setMapPC(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc_);
    ~CloudInitializer();
    void showImgIni(double x,double y,int yaw);
    // void MAIN();
    // during rescue, get possible robot pose,save possible pose in a vector
    void getInitialExtGuess(const sensor_msgs::PointCloudConstPtr& laserCloudMsg);
    // void getInitialExtGuess(const sensor_msgs::PointCloudConstPtr& laserCloudMsg,Eigen::Matrix4f &robotPose );
    // void getInitialExtGuess( sensor_msgs::PointCloud laserCloudMsg);
    void rescueRobot();
    void scoreParticlesDist();
    CloudInitializer();
    // CloudInitializer(pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing_,pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc_,std_msgs::Header mapHeader_,ros::NodeHandle& _nh);
    // CloudInitializer(ros::NodeHandle& _nh);
    
    void scoreParticles();
    void checkingGuess();
    void checkWholeMap(const pcl::PointXYZI& PCPoint,const pcl::PointXYZI &PosePoint,int horizonIndex,double & minDist,bool& findIntersection);
    // given two pc, and robot pose, calculate their score
    double getScoreFromTwoPC(const Eigen::Matrix4f & robotPose, pcl::PointCloud<pcl::PointXYZI>::Ptr PC1,pcl::PointCloud<pcl::PointXYZI>::Ptr  PC2 );
    void calClosestMapPoint(int inside_index) override;
    bool checkMap(int ring, int horizonIndex, int &last_index, double & minDist,int inside_index) override;
    void allocateMemory() override;
    void resetParameters() override;
    void initializationICP(int insideAGIndex);
    bool checkICPmovingDist(Eigen::Matrix4f robotPoseGuess);
    bool insideOldArea(int mapPCindex);

};
#endif