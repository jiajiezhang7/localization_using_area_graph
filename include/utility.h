#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/point_cloud_conversion.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include<pcl/common/pca.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include<stdlib.h>
#include<stdio.h>
#include <ceres/ceres.h>
#include <chrono>
#include<typeinfo>
#include "areaGraphDataParser/AGindex.h"
#include "areaGraphDataParser/areaIndex.h"

using namespace std;

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D        // 位置
        PCL_ADD_INTENSITY; // 激光点反射强度，也可以存点的索引
    uint16_t ring;         // 扫描线
    // float time;         // 时间戳，记录相对于当前帧第一个激光点的时差，第一个点time=0
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16; // 内存16字节对齐，EIGEN SSE优化要求
// 注册为PCL点云格式
// POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint16_t, ring, ring) (float, time, time)
// )
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))
// may be needed for deskew
using PointXYZIRT = VelodynePointXYZIRT;


typedef pcl::PointXYZI PointType;

// 传感器型号
enum class SensorType { VELODYNE, OUSTER };
class ParamServer
{
public:

    ros::NodeHandle nh;
    string pointCloudTopic; // points_raw 原始点云数据
    // 坐标系
    string lidarFrame;      // 激光坐标系
    string baselinkFrame;   // 载体坐标系
    string odometryFrame;   // 里程计坐标系
    string mapFrame;        // 世界坐标系

    // 激光传感器参数
    // SensorType sensor;      // 传感器型号
    int N_SCAN;             // 扫描线数，例如16、64
    int Horizon_SCAN;       // 扫描一周计数，例如每隔0.2°扫描一次，一周360°可以扫描1800次
    int downsampleRate;     // 扫描线降采样，跳过一些扫描线
    int downsampleRateHorizontal; //跳过一圈中的一些点
    float lidarMinRange;    // 最小范围
    float lidarMaxRange;    // 最大范围
    int N_ceiling;

    vector<float> extRotV;
    // vector<double> extRPYV;
    vector<float> extTransV;
    vector<float> mapextTransV;

    Eigen::Matrix3f initialExtRot;     // xyz坐标系旋转
    // Eigen::Matrix3d extRPY;     // RPY欧拉角的变换关系
    Eigen::Vector3f initialExtTrans;   // xyz坐标系平移
    Eigen::Vector3f mapExtTrans;   // xyz坐标系平移

    float initialYawAngle;
    float mapYawAngle;

    // Eigen::Quaterniond extQRPY;

    //optimization para
    double errorUpThredInit;
    double errorLowThredInit;
    //optimization para
    double errorUpThred;
    double errorLowThred;
    bool opti;
    //LBFGS para
    int mem_size;
    double g_epsilon;
    int past;
    double delta;
    int max_iterations;
    int max_linesearch;
    double min_step;
    double max_step;
    double f_dec_coeff; 
    double s_curv_coeff;
    double cautious_factor;
    double machine_prec;

    //icp opti para
    double translation_thres;
    // maximum iteration
    int icp_iteration;
    // initialization maximum iteration
    int icp_init_iteration;
    bool use_weight;
    bool detect_corridor;
    double maxPercentageCorridor;
    double  recalIntersectionThred;
    double percentageThred;
    double averDistanceThred;
    double radiusDisthred;
    double groundThred;
    double ceilingThred;
    double parallelThred;
    int subSample;
    //pause to check intialization
    bool pause_iter;
    bool initialization_imu;
    double diff_angle_init;
    double icp_stop_translation_thred;
    double icp_stop_rotation_thred;
    double rescue_angle_interval;
    bool bRescueRobot;
    bool bTestRescue;
    bool bOnlyScoreParticles;
    bool bResultChecking;
    double scoreDownsampleRate;

    double checkingAngle;
    double checkingGuessX;
    double checkingGuessY;

    bool bGenerateResultFile;
    bool bFurthestRingTracking;
    double turkeyPauseThred;
    double corridorDSmaxDist;

    bool bAllPassageOpen;
    bool bAllPassageClose;
    bool bInitializationWithICP;


    ParamServer()
    {

        nh.param<std::string>("pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<int>("N_SCAN", N_SCAN, 32);
        nh.param<int>("Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("downsampleRate", downsampleRate, 1);
        nh.param<float>("lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lidarMaxRange", lidarMaxRange, 1000.0);
        nh.param<int>("N_ceiling", N_ceiling, 32);
        nh.param<int>("downsampleRateHorizontal", downsampleRateHorizontal, 1);

        nh.param<vector<float>>("initialExtrinsicRot", extRotV, vector<float>());
        nh.param<vector<float>>("initialExtrinsicTrans", extTransV, vector<float>());
        nh.param<vector<float>>("mapExtrinsicTrans", mapextTransV, vector<float>());

        nh.param<float>("initialYawAngle", initialYawAngle, 0);
        nh.param<float>("mapYawAngle", mapYawAngle, 0);

        initialExtRot = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        initialExtTrans = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        mapExtTrans = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(mapextTransV.data(), 3, 1);
        
        nh.param<double>("errorUpThredInit", errorUpThredInit, 0);
        nh.param<double>("errorLowThredInit", errorLowThredInit, 0);
        nh.param<double>("errorUpThred", errorUpThred, 0);
        nh.param<double>("errorLowThred", errorLowThred, 0);

        nh.param<bool>("opti",opti,false);

        // LBFGS para
        // nh.param<int>("mem_size",mem_size,8);
        // nh.param<double>("g_epsilon",g_epsilon,0.0);
        // nh.param<int>("past",past,3);
        // nh.param<double>("delta",delta,1.0e-6);
        // nh.param<int>("max_iterations",max_iterations,0);
        // nh.param<int>("max_linesearch",max_linesearch,64);
        // nh.param<double>("min_step",min_step,1.0e-20);
        // nh.param<double>("max_step",max_step,1.0);
        // nh.param<double>("f_dec_coeff",f_dec_coeff,1.0e-4);
        // nh.param<double>("s_curv_coeff",s_curv_coeff,0.9);
        // nh.param<double>("cautious_factor",cautious_factor,1.0e-6);
        // nh.param<double>("machine_prec",machine_prec,1.0e-16);
        
        // icp  opti para
        nh.param<double>("translation_thres",translation_thres,1.0e-6);
        nh.param<int>("icp_iteration",icp_iteration,5);
        nh.param<int>("icp_init_iteration",icp_init_iteration,50);

        nh.param<bool>("use_weight",use_weight,false);
        nh.param<double>("recalIntersectionThred",recalIntersectionThred,5);
        nh.param<double>("percentageThred",percentageThred,30.0);
        nh.param<double>("averDistanceThred",averDistanceThred,1.0);
        nh.param<double>("radiusDisthred",radiusDisthred,0.1);
        nh.param<double>("groundThred",groundThred,-0.75);
        nh.param<double>("ceilingThred",ceilingThred,1.8);
        nh.param<double>("parallelThred",parallelThred,15.0);
        nh.param<int>("subSample",subSample,2);

        nh.param<bool>("pause_iter",pause_iter,false);
        nh.param<bool>("initialization_imu",initialization_imu,true);
        nh.param<double>("diff_angle_init",diff_angle_init,15.0);
        nh.param<double>("icp_stop_translation_thred",icp_stop_translation_thred,0.01);
        nh.param<double>("icp_stop_rotation_thred",icp_stop_rotation_thred,0.01);
        nh.param<bool>("detect_corridor",detect_corridor,true);
        nh.param<double>("maxPercentageCorridor",maxPercentageCorridor,1);

        nh.param<double>("rescue_angle_interval",rescue_angle_interval,15);
        nh.param<bool>("bRescueRobot",bRescueRobot,false);
        nh.param<bool>("bTestRescue",bTestRescue,false);
        nh.param<bool>("bOnlyScoreParticles",bOnlyScoreParticles,false);
        nh.param<double>("scoreDownsampleRate",scoreDownsampleRate,0.1);
        nh.param<bool>("bResultChecking",bResultChecking,false);
        nh.param<double>("checkingAngle",checkingAngle,0.1);
        nh.param<double>("checkingGuessX",checkingGuessX,0.1);
        nh.param<double>("checkingGuessY",checkingGuessY,0.1);
        nh.param<bool>("bGenerateResultFile",bGenerateResultFile,false);
        nh.param<bool>("bFurthestRingTracking",bFurthestRingTracking,false);
        nh.param<double>("turkeyPauseThred",turkeyPauseThred,140);
        nh.param<double>("corridorDSmaxDist",corridorDSmaxDist,8);
        nh.param<bool>("bAllPassageOpen",bAllPassageOpen,false);
        nh.param<bool>("bAllPassageClose",bAllPassageClose,false);

        nh.param<bool>("bInitializationWithICP",bInitializationWithICP,false);

        usleep(100);
    }

    void calPedal(double x1,  double y1, double x2, double y2,double x3, double y3,double & x4, double & y4){
    x4=(x1*x1*x3 - 2*x1*x2*x3 - x1*y1*y2 + x1*y1*y3 + x1*y2*y2 - x1*y2*y3 + x2*x2*x3 + x2*y1*y1 - x2*y1*y2 - x2*y1*y3 + x2*y2*y3)/(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2);
    y4= (x1*x1*y2 - x1*x2*y1 - x1*x2*y2 + x1*x3*y1 - x1*x3*y2 + x2*x2*y1 - x2*x3*y1 + x2*x3*y2 + y1*y1*y3 - 2*y1*y2*y3 + y2*y2*y3)/(x1*x1- 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2);
    }

    // p1 p2为一条直线，p3,p4为一条直线
    pcl::PointXYZI calIntersection(pcl::PointXYZI p1,pcl::PointXYZI p2,pcl::PointXYZI p3,pcl::PointXYZI p4){
        pcl::PointXYZI intersection;
        intersection.x=(p3.y*p4.x*p2.x-p4.y*p3.x*p2.x-p3.y*p4.x*p1.x+p4.y*p3.x*p1.x-p1.y*p2.x*p4.x+p2.y*p1.x*p4.x+p1.y*p2.x*p3.x-p2.y*p1.x*p3.x)/(p4.x*p2.y-p4.x*p1.y-p3.x*p2.y+p3.x*p1.y-p2.x*p4.y+p2.x*p3.y+p1.x*p4.y-p1.x*p3.y);
        intersection.y=(-p3.y*p4.x*p2.y+p4.y*p3.x*p2.y+p3.y*p4.x*p1.y-p4.y*p3.x*p1.y+p1.y*p2.x*p4.y-p1.y*p2.x*p3.y-p2.y*p1.x*p4.y+p2.y*p1.x*p3.y)/(p4.y*p2.x-p4.y*p1.x-p3.y*p2.x+p1.x*p3.y-p2.y*p4.x+p2.y*p3.x+p1.y*p4.x-p1.y*p3.x);
        intersection.z=p1.z;
        return intersection;
    }

    //默认P1是robot pose，P2为世界坐标系下激光点，P3，P4为地图点
    //激光线与地图线的交点应该在地图线上
    //激光线与pose与intersection的向量夹角应为锐角
    //p3,p4顺序无要求
    //change intersection 这条激光线与地图的交点
    //TODO: intersection 不存在的点置零
    //计算map与激光点射线有几个交点， 交点在map line上并且交点在robot pose与激光点之间
    //used in RAY TRACING
    bool inBetween(pcl::PointXYZI p1,pcl::PointXYZI p2,pcl::PointXYZI p3,pcl::PointXYZI p4,pcl::PointXYZI * intersection){
    //不处理激光点为（0，0）的点
        // double distance =calDistance(p1,p2);//sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
        //not parallel
        if((p4.x*p2.y-p4.x*p1.y-p3.x*p2.y+p3.x*p1.y-p2.x*p4.y+p2.x*p3.y+p1.x*p4.y-p1.x*p3.y)!=0
        &&(p4.y*p2.x-p4.y*p1.x-p3.y*p2.x+p1.x*p3.y-p2.y*p4.x+p2.y*p3.x+p1.y*p4.x-p1.y*p3.x)!=0){

        // if((distance > lidarMinRange && distance < lidarMaxRange)&&(p4.x*p2.y-p4.x*p1.y-p3.x*p2.y+p3.x*p1.y-p2.x*p4.y+p2.x*p3.y+p1.x*p4.y-p1.x*p3.y)!=0&&(p4.y*p2.x-p4.y*p1.x-p3.y*p2.x+p1.x*p3.y-p2.y*p4.x+p2.y*p3.x+p1.y*p4.x-p1.y*p3.x)!=0){
            pcl::PointXYZI intersection_=calIntersection(p1,p2,p3,p4);
            double innerproduct=(intersection_.x-p1.x)*(p2.x-p1.x)+(intersection_.y-p1.y)*(p2.y-p1.y);
            if( innerproduct>=0&&((p4.x<=intersection_.x&&intersection_.x<=p3.x)||(p3.x<=intersection_.x&&intersection_.x<=p4.x))){
                *intersection=intersection_;
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }

    //on ray:https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    //intersection in both lidar line and map line
    void inRay(pcl::PointXYZI p1,pcl::PointXYZI p2,pcl::PointXYZI p3,pcl::PointXYZI p4, bool &bOnRay){
        //不处理激光点为（0，0）的点
        bOnRay=false;
        double distance =sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
        //FIXME: filter should happen before 
        if((distance > lidarMinRange && distance < lidarMaxRange)&&
        (p4.x*p2.y-p4.x*p1.y-p3.x*p2.y+p3.x*p1.y-p2.x*p4.y+p2.x*p3.y+p1.x*p4.y-p1.x*p3.y)!=0
        &&(p4.y*p2.x-p4.y*p1.x-p3.y*p2.x+p1.x*p3.y-p2.y*p4.x+p2.y*p3.x+p1.y*p4.x-p1.y*p3.x)!=0){
            pcl::PointXYZI intersection_=calIntersection(p1,p2,p3,p4);
            if(((p4.x<=intersection_.x&&intersection_.x<=p3.x)||(p3.x<=intersection_.x&&intersection_.x<=p4.x))&&
                ((p2.x<=intersection_.x&&intersection_.x<=p1.x)||(p1.x<=intersection_.x&&intersection_.x<=p2.x))){
                bOnRay=true;
            }
        }
        else{
            cout<<"should not filter"<<endl;
        }
    }

    void inRayGeneral(pcl::PointXYZI p1,pcl::PointXYZI p2,pcl::PointXYZI p3,pcl::PointXYZI p4, bool &bOnRay){
        bOnRay=false;
        // double distance =sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
        //FIXME: filter should happen before 
        if((p4.x*p2.y-p4.x*p1.y-p3.x*p2.y+p3.x*p1.y-p2.x*p4.y+p2.x*p3.y+p1.x*p4.y-p1.x*p3.y)!=0
        &&(p4.y*p2.x-p4.y*p1.x-p3.y*p2.x+p1.x*p3.y-p2.y*p4.x+p2.y*p3.x+p1.y*p4.x-p1.y*p3.x)!=0){
            pcl::PointXYZI intersection_=calIntersection(p1,p2,p3,p4);
            if(((p4.x<=intersection_.x&&intersection_.x<=p3.x)||(p3.x<=intersection_.x&&intersection_.x<=p4.x))&&
                ((p2.x<=intersection_.x&&intersection_.x<=p1.x)||(p1.x<=intersection_.x&&intersection_.x<=p2.x))){
                bOnRay=true;
            }
        }
        else{
            cout<<"should not filter"<<endl;
        }
    }

    double calWeightTurkey(double r, double k,bool outside,double outsideThred){
        // return (1-r*r/k*k)*(1-r*r/k*k);
        if(outside){
            // if(r<1.1){
            //     return-1.60897837*r*r*r+1.48694532*r*r-0.4158812*r+1.000683;
            // }
            // else{
            //     return -0.001537*r*r*r + 0.02218863*r*r-0.17013104 *r+ 0.36189266;
            // }
            // go through (0,1) and (threshold,0.1)
            if(r>outsideThred){
                return 0;
            }
            else{
                return outsideThred/(9*r+outsideThred);
            }
        }else{
            
            if(r>k){
                return 0;
            }
            else{
                return k/(1.5*r+k);
            }
        }
    }

    double calWeightHuber(double r, double k){
        return 0;
    }

    double calWeightCauchy(double r, double k){
        return 1/(1+r*r/k/k);
    }

    double checkParallel(pcl::PointXYZI p1,pcl::PointXYZI p2,pcl::PointXYZI p3,pcl::PointXYZI p4){
        double angle=acos(((p1.x-p2.x)*(p3.x-p4.x)+(p1.y-p2.y)*(p3.y-p4.y))/sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y))/sqrt((p3.x-p4.x)*(p3.x-p4.x)+(p3.y-p4.y)*(p3.y-p4.y)))/M_PI*180;
        return angle;
    }

    void getPCA(Eigen::Vector3f & eigen_values,  Eigen::Matrix3f & eigen_vector, pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing, bool &bPCA ){
        pcl::PCA<pcl::PointXYZI>pca;
        pca.setInputCloud(furthestRing);
        eigen_values=pca.getEigenValues();
        eigen_vector=pca.getEigenVectors();
        std::cout << "pca eigen_values"<<eigen_values(1)/eigen_values(0) << std::endl;

        std::cout << "pca eigen_vector"<<eigen_vector << std::endl;
        bPCA=true;
    }

    double calDistance(pcl::PointXYZI p1,pcl::PointXYZI p2){
        return std::sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
    }
    
    // distance from p0 to line p1-p2
    double calDistance2Line(pcl::PointXYZI p0,pcl::PointXYZI p1,pcl::PointXYZI p2){
        double k=(p1.y-p2.y)/(p1.x-p2.x);
        double distance=std::abs(k*p0.x-p0.y+p1.y-k*p1.x)/std::sqrt(k*k+1);
        return distance;
    }

};

#endif

