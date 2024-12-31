#pragma onceCloudBaserobotPose
#ifndef _CLOUD_BASE_HPP_
#define _CLOUD_BASE_HPP_
#include "utility.h"

class CloudBase : public ParamServer
{
public:
    ros::Subscriber subMap;
    ros::Subscriber subMapAG;

    ros::Subscriber subMapInit;
    ros::Subscriber subImu;
    ros::Subscriber subLiosamPath;
    ros::Subscriber subLiosamodometry_incremental;

    // ros::Subscriber subCorridorEnlarge;
    ros::Subscriber subAGindex;
    // for publish
    std_msgs::Header cloudHeader;
    std_msgs::Header mapHeader;
    nav_msgs::Path globalPath;

    ros::Publisher pubUppestRing;
    ros::Publisher pubFurthestRing;
    ros::Publisher pubPotentialCeiling;
    ros::Publisher pubtest;
    // publish organized pointcloud
    ros::Publisher pubOrganizedCloudIn;
    // publish this ray 's intersection with map
    ros::Publisher pubIntersection;
    // publish pointcloud that has been transformed to map frame
    ros::Publisher pubTransformedPC;
    ros::Publisher pubTransformedWholePC;

    // useful lidar pointcloud for opti, rule out clutter, etc,
    ros::Publisher pubUsefulPoints1;
    // useful map intersection for opti
    ros::Publisher pubUsefulPoints2;
    // puiblish map that has been subscribe
    ros::Publisher pubMapPC;
    ros::Publisher pubAGMapTransformedPC;

    // ros::Publisher  pubMapCorridorEnlargePC;

    // publish optimized pose's PC
    ros::Publisher pubOptiPC;
    ros::Publisher pubRobotPath;
    ros::Publisher pubTransformedLiosamPath;
    // send to node initDataSender to ask for another frame of PC
    ros::Publisher pubDONEsignal;
    ros::Publisher pubInsidePC;
    ros::Publisher pubOutsidePC;
    ros::Publisher pubinfinity;


    nav_msgs::Path TransformedLiosamPath;

    bool bUseFurestestRing;
    // hope to use PCA to tell if the robot is inside corridor when rescue
    bool bPCA;
    bool AGindexReceived;
    // record current area index the robot is in
    int lastInsideIndex;

    double insideScore;
    double outsideScore;
    double insideTotalRange;
    double outsideTotalScore;

    /*
    organizedCloudIn size:
    Horizon_SCAN:
    N_SCAN*Horizon_SCAN: initialized or case3 initialization
     */
    /*
    three cases:
    reset every frame:
            laserCloudIn
            organizedCloudIn
            furthestRing
    reset every guess:
            transformed_pc
            ringMapP1
            ringMapP2
            intersectionOnMap

            insidePC
            outsidePC
    reset every icp iteration:
            UsefulPoints1
            UsefulPoints2

    */
    // raw pointcloud from msg
    // laserCloudIn->organizedCloudIn->transformed_pc
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    // 一维数组，按顺序存所有的点, 行为ring，列为第几个点，从ring0到ring31，ring0是最低的那一圈
    // points organized by ring and angle, 1d array, N_SCAN*Horizon_SCAN, maybe change lidar mode to first return mode, to rule out points
    // intensity is point distance
    pcl::PointCloud<pcl::PointXYZI>::Ptr organizedCloudIn;
    pcl::PointCloud<pcl::PointXYZI>::Ptr organizedCloudIn64;

    // use for rescue robot, only use the furthest points
    pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing;
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedFurthestRing;

    // 转换过pose的organized pc
    // organized pointcloud in map frame
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc;
    // 地图点pointcloud,map pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapCorridorEnlarge_pc;

    // 记录PC一圈中每个点对应的最近的map点
    // lidar pointcloud use which two map points to form a line and intersect with
    pcl::PointCloud<pcl::PointXYZI>::Ptr ringMapP1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ringMapP2;
    // coordinate of supposed interestion between map line and lidar ring
    pcl::PointCloud<pcl::PointXYZI>::Ptr intersectionOnMap;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr intersectionMapIndexRecord;

    // useful lidar pointcloud for opti, rule out clutter, etc,
    // usefulpoint.intensity means match with which map line
    pcl::PointCloud<pcl::PointXYZI>::Ptr UsefulPoints1;
    // useful map interestion for opti
    pcl::PointCloud<pcl::PointXYZI>::Ptr UsefulPoints2;
    // not rule out points explicitly anymore
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserUppestRing;
    pcl::PointCloud<pcl::PointXYZI>::Ptr potentialCeilingPoints;
    // pointcloud and inside map and outside map,
    // maybe count insidePC number and outsidePC number to determinate if that is a possible place instead icp to save computation
    // set every guess
    // only for visualization, which points are inside and outside current area
    pcl::PointCloud<pcl::PointXYZI>::Ptr insidePC;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outsidePC;

    // size of map pointcloud
    int mapSize;
    // if the map is initialized, if not initialized, don't process pointcloud
    bool mapInit;
    std::deque<sensor_msgs::Imu> imu_buf;
    // T_wl
    // Eigen::Affine3f robotPose;
    Eigen::Matrix4f robotPose;

    // map center of mass
    Eigen::Vector2d mapCenter;
    // PC center of mass
    Eigen::Vector2d PCCenter;
    // number of useful points for icp
    int numIcpPoints;
    double IcpPointsPercentage;
    // how many times algorithm receive map
    int mapReceivedTimes;
    // average distance of paired useful points
    double averDistancePairedPoints;

    Eigen::Vector2d mapCenterInitialization;
    Eigen::Vector2d PCCenterInitialization;

    // double mapCenterWeight;
    // record index of UsefulPoints, so that when localize in corridor, the dominate points canbe decreased...
    std::vector<int> usefulIndex;
    std::vector<int> outsideAreaIndexRecord;
    std::vector<int> outsideAreaLastRingIndexRecord;

    // curent useful points threshold, if in rescue mode, bigger than in  localization mode
    // outside
    double errorUpThredCurr;
    // inside
    double errorLowThredCurr;
    // record all weight sum
    double weightSumTurkey;
    double weightSumCauchy;
    std::vector<double> weightsTurkey;
    // record how mant angle has been changed for one frame, if too big, need to recaluate useful points,
    // since intersection may jump to other map line
    double accumulateAngle;
    bool initialized;
    // if localize inside corridor
    bool onlyOneDirection;
    // 与map大小相同，随pointcloud每一帧变化, 记录每条mapline有多少个匹配点, in order to determinate if the robot is inside a corridor
    std::vector<int> mapHistogram;
    // useful points 中transformed pointcloud的x坐标
    std::vector<double> Vec_pcx;
    std::vector<double> Vec_pcy;
    // useful points 中map points的x坐标
    std::vector<double> Vec_pedalx;
    std::vector<double> Vec_pedaly;
    // during rescue, particle location
    std::vector<Eigen::Vector3f> corridorGuess;
    std::vector<Eigen::Vector3f> roomGuess;
    areaGraphDataParser::AGindex AG_index;
    
    // histogram 中一共有多少个点
    int numTotalHistogram;
    int currentIteration;

    // run how many frames, useless
    int runTime;
    // record result
    ofstream GTstream;
    ofstream robotPoseTum;
    ofstream LiosamPoseTum;

    CloudBase();
    // pointcloud callback
    // void cloudHandlerCB(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    // only to get GT
    void liosamPathCB(const nav_msgs::Path &pathMsg);
    // BASE
    void mapCB(const sensor_msgs::PointCloudConstPtr &laserCloudMsg);
    /* 
    organizePointcloud
    // 搜索所有粒子位置
    scoreParticlesDist();
            setInitialPose
            // 搜索一圈所有激光点
            calClosestMapPoint
                // 搜索所有地图
                checkMap
                    //两点和两地图点
                    inBetween
            resetParameters
     */
    void organizePointcloud();
    // set robot pose according to params.yaml
    void setInitialPose(double initialYawAngle, Eigen::Vector3f initialExtTrans);
    void pubPclCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, ros::Publisher *pub, std_msgs::Header *cloudHeader);
    // set params every frame
    void setEveryFrame();
    void mapAGCB(const sensor_msgs::PointCloudConstPtr &laserCloudMsg);
    // void AGindexCB(const areaGraphDataParser::AGindexConstPtr& msg);
    void AGindexCB(const areaGraphDataParser::AGindex& msg);
    bool areaInsideChecking(const Eigen::Matrix4f& robotPose, int areaStartIndex);
    geometry_msgs::PoseStamped transformLiosamPath(const nav_msgs::Path& pathMsg);
    geometry_msgs::Pose transformLiosamPathnew (const nav_msgs::Odometry::ConstPtr pathMsg);

    // END OF BASE

    // if the intersection is actually on the map
    // bool inBetween(pcl::PointXYZI p1, pcl::PointXYZI p2, pcl::PointXYZI p3, pcl::PointXYZI p4, pcl::PointXYZI *intersection);
    // if the intersection between p1p2 and p3p4 is in the middle of these lines
    // void inRay(pcl::PointXYZI p1, pcl::PointXYZI p2, pcl::PointXYZI p3, pcl::PointXYZI p4, bool &bOnRay);

    //VIRTUAL 
    // each ray is supposed to intersect with map, cal the supposed intersection with map according to robot pose and map
    virtual void calClosestMapPoint(int inside_index);
    // only process one column
    virtual bool checkMap(int ring, int horizonIndex, int &last_index, double & minDist,int inside_index);
    virtual void allocateMemory();
    // during rescue, organized pointcloud donot need to be reset after each guess
    virtual void resetParameters();
    // END OF VIRTUAL


    void initializedUsingMassCenter();
    // first frame of lidar, initialization
    // void rescueRobot();
    // during rescue, get possible robot pose,save possible pose in a vector


    void getGTfromLiosam(std_msgs::Header cloudHeader);
    // during rescue, only use the furthest points of all rings to save computation time
    void formFurthestRing();
    double calParticleDist(int ring, int horizonIndex, int &last_index);
    void saveTUMTraj(geometry_msgs::PoseStamped& pose_stamped);
    // to check if the robot pose is inside a certain area, mainly used in nomal localization,
    //since the robot may move from one area to another
    // END OF NOT USED
};

#endif