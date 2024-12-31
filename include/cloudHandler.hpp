#pragma once
#ifndef _CLOUD_HANDLER_HPP_
#define _CLOUD_HANDLER_HPP_
#include "utility.h"
#include "cloudBase.hpp"
#include "cloudInitializer.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cloudInitializer.hpp"

class CloudHandler : public CloudBase
{
public:
    ros::Subscriber subLaserCloud;
    // 发布机器人所在Area的node点
    ros::Publisher pubinsideAreaPC;
    ros::Subscriber subInitialGuess;
    
    CloudInitializer cloudInitializer;
    // MAP PC index
    int insideAreaStartIndex;
    // data structure AGindex index
    int insideAreaID;
    std::vector<bool> vbHistogramRemain;
    clock_t sumFrameRunTime;
    int numofFrame;
    bool getGuessOnce;
    int globalImgTimes;
    CloudHandler();
    void filterUsefulPoints();
    void optimizationICP();
    void showImg1line(string words);

    void cloudHandlerCB(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void liosamOdometryIncrementalCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    
    void mergeMapHistogram();
    double corridornessDSRate(double maxPercentage);
    //getting which area the robotPose is inside right now.
    void gettingInsideWhichArea();
    bool checkWholeMap(int pc_index,const pcl::PointXYZI& PCPoint,double &map1x,double &map1y,double &map2x,double &map2y,double & intersectionx, double & intersectiony);

    void calClosestMapPoint(int inside_index) override;
    bool checkMap(int ring, int horizonIndex, int &last_index, double & minDist,int inside_index) override;
    void allocateMemory() override;
    void resetParameters() override;
    void getInitialExtGuess(const sensor_msgs::PointCloudConstPtr& laserCloudMsg);

};

#endif