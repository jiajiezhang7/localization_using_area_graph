#include "cloudBase.hpp"
#include "utility.h"

void CloudBase::saveTUMTraj(geometry_msgs::PoseStamped & pose_stamped){
        robotPoseTum<<pose_stamped.header.stamp.toSec()<<" "<<pose_stamped.pose.position.x<<" "<<pose_stamped.pose.position.y<<" "<<pose_stamped.pose.position.z<<" "<<pose_stamped.pose.orientation.x<<" "<<pose_stamped.pose.orientation.y<<" "<<pose_stamped.pose.orientation.z<<" "<<pose_stamped.pose.orientation.w<<std::endl;
}
    
// check if the robot is inside this area
bool CloudBase::areaInsideChecking(const Eigen::Matrix4f& robotPose,int areaStartIndex){
    pcl::PointXYZI robot;
    robot.x=robotPose(0,3);
    robot.y=robotPose(1,3);
    robot.z=robotPose(2,3);
    robot.intensity=0;
    pcl::PointXYZI robotInfinity;
    robotInfinity.x=robotPose(0,3)+5132;
    robotInfinity.y=robotPose(1,3)+2345;
    robotInfinity.z=robotPose(2,3);
    robotInfinity.intensity=0;

    int throughTimes=0;

    visualization_msgs::Marker points, line_strip, line_list;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.header=mapHeader;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    line_strip.scale.x = 0.1;
    line_strip.id = 1;
    line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    // check all map lines of this area, record how many times a ray goes through map polygon to decide if the robot is inside or outside of this area
    for(int i=areaStartIndex;i<areaStartIndex+1000000;i++){
        // the end of this area
        if((int)map_pc->points[i].intensity%3==2){
            break;
        }
        bool b_inray;
        inRayGeneral(map_pc->points[i],map_pc->points[(i+1)%mapSize],robot,robotInfinity, b_inray);
        if(b_inray){
            throughTimes++;
        }
    }
    //inside this area
    if(throughTimes%2==1){
        // cout<<" through times = "<<throughTimes<<endl;
        geometry_msgs::Point p;
        p.x = robot.x;
        p.y = robot.y;
        p.z = robot.z;
        geometry_msgs::Point p_;
        p_.x = robotInfinity.x;
        p_.y = robotInfinity.y;
        p_.z = robotInfinity.z;
        line_strip.points.push_back(p);
        line_strip.points.push_back(p_);
        pubinfinity.publish(line_strip);

        return true;
    }
    else{
        return false;
    }
}

CloudBase::CloudBase(){
    // 地图相关
    mapInit=false;           // 地图是否初始化
    mapCenter.setZero();     // 地图中心点
    PCCenter.setZero();      // 点云中心点
    mapReceivedTimes=0;      // 地图接收次数
    AGindexReceived=false;   // 是否接收到Areaindex
    lastInsideIndex=-1;      // 上一次所在区域的索引

    // ICP相关
    numIcpPoints=0;          // icp点的数量
    IcpPointsPercentage=0;   // 用于icp的点的百分比
    accumulateAngle=0;       // 累计角度
    initialized=false;        // 是否初始化完成
    onlyOneDirection=false;   // 是否只有一个方向

    // GTstream.open("/home/xiefujing/research/area_graph/ws/GT/GTliosam2023-05-10-20-16-52.txt", ofstream::app);
    GTstream.open("/home/xiefujing/research/area_graph/ws/GT/GTliosam2023-05-24-20-54-47.txt", ofstream::app);
    GTstream.setf(ios::fixed);
    GTstream.precision(2);

    // allocateMemory();
    // temporary, get initial guess out of a file
    // getInitialExtGuess();

    ros::Time current_time = ros::Time::now();

    TransformedLiosamPath.header.stamp=current_time;
    TransformedLiosamPath.header.frame_id="/map";

    // subMap        = nh.subscribe<sensor_msgs::PointCloud>("/mapPC",1,&CloudBase::mapCB, this, ros::TransportHints().tcpNoDelay());
    // 订阅地图话题
    subMapAG    = nh.subscribe<sensor_msgs::PointCloud>("/mapPC_AG",1,&CloudBase::mapAGCB, this, ros::TransportHints().tcpNoDelay());
    
    //sub in cloud handler
    // subLiosamPath=nh.subscribe("/lio_sam/mapping/odometry",1000,&CloudBase::liosamPathCB, this, ros::TransportHints().tcpNoDelay());
    // subCorridorEnlarge=nh.subscribe<sensor_msgs::PointCloud>("/corridorEnlargePC",1,&CloudBase::mapCorrdorEnlargeCB, this, ros::TransportHints().tcpNoDelay());
    // subAGindex= nh.subscribe<areaGraphDataParser::AGindex>("/AGindex",1000,&CloudBase::AGindexCB, this, ros::TransportHints().tcpNoDelay());
    // 订阅区域图索引
    subAGindex= nh.subscribe("/AGindex",1000,&CloudBase::AGindexCB, this, ros::TransportHints().tcpNoDelay());
    
    // 发布话题（大量的可视化话题）
    pubRobotPath                     = nh.advertise<nav_msgs::Path>("RobotPath", 1);           // 机器人路径
    pubUppestRing = nh.advertise<sensor_msgs::PointCloud2> ("uppestRing", 1);                  // 可视化：最上层的点云环
    pubFurthestRing = nh.advertise<sensor_msgs::PointCloud2> ("FurthestRing", 1);              // 可视化：最远的点云环
    pubPotentialCeiling = nh.advertise<sensor_msgs::PointCloud2> ("potentialCeiling", 1);      // 可视化：最可能的天花板
    pubOrganizedCloudIn= nh.advertise<sensor_msgs::PointCloud2> ("pubOrganizedCloudIn", 1);    // 可视化：有序的点云
    pubMapPC= nh.advertise<sensor_msgs::PointCloud2> ("pubMapPC", 1);                          // 可视化：地图
    pubAGMapTransformedPC= nh.advertise<sensor_msgs::PointCloud2> ("pubAGMapTransformedPC", 1);// TODO 还不知道这是啥

    // pubMapCorridorEnlargePC= nh.advertise<sensor_msgs::PointCloud2> ("pubMapCorridorEnlargePC", 1);

    pubIntersection= nh.advertise<sensor_msgs::PointCloud2> ("pubIntersection", 1);            // 可视化：激光点与地图的交点
    pubTransformedPC= nh.advertise<sensor_msgs::PointCloud2> ("pubTransformedPC", 1);          // 可视化：转换后的点云
    pubTransformedWholePC= nh.advertise<sensor_msgs::PointCloud2> ("pubTransformedWholePC", 1);// 可视化：转换后的整个点云 TODO 还不知道这是啥

    // TODO 还不明白以下两个Useful的意义
    pubUsefulPoints1= nh.advertise<sensor_msgs::PointCloud2> ("pubUsefulPoints1", 1);  
    pubUsefulPoints2= nh.advertise<sensor_msgs::PointCloud2> ("pubUsefulPoints2", 1);

    pubtest = nh.advertise<sensor_msgs::PointCloud2> ("pubtest", 1);
    pubOptiPC = nh.advertise<sensor_msgs::PointCloud2> ("pubOptiPC", 1);
    pubInsidePC= nh.advertise<sensor_msgs::PointCloud2> ("pubInsidePC", 1);
    pubOutsidePC= nh.advertise<sensor_msgs::PointCloud2> ("pubOutsidePC", 1);

    pubTransformedLiosamPath = nh.advertise<nav_msgs::Path> ("TransformedLiosamPath", 1);
    pubDONEsignal=nh.advertise<geometry_msgs::Pose>("doneInit",1);
    pubinfinity = nh.advertise<visualization_msgs::Marker>("pubinfinity", 10);

    //out of param.yaml
    // 从yaml文件设置初始位姿
    setInitialPose(initialYawAngle,initialExtTrans);
}

// only call this for 2022-11-14-19-07-34 bag
geometry_msgs::PoseStamped CloudBase::transformLiosamPath(const nav_msgs::Path& pathMsg){
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped=*(pathMsg.poses.end()-1);
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix<<0.9965  ,	-0.0800   ,	-0.0080,
                                            0.0800  , 	0.9966   ,	-0.0006,
                                            0.0080  , 	-0.0006   	,0.9999;

    Eigen::Quaterniond quaternion(rotation_matrix);
    Eigen::Quaterniond quaternionLIO(this_pose_stamped.pose.orientation.w,this_pose_stamped.pose.orientation.x,this_pose_stamped.pose.orientation.y,this_pose_stamped.pose.orientation.z);
    Eigen::Quaterniond afterRotationQuat=quaternion*quaternionLIO;
    double newx= 0.9965*(pathMsg.poses.end()-1)->pose.position.x - 0.08*	(pathMsg.poses.end()-1)->pose.position.y-0.0080*(pathMsg.poses.end()-1)->pose.position.z - 0.1024  ;
    double newy=0.0800*(pathMsg.poses.end()-1)->pose.position.x+0.9966*	(pathMsg.poses.end()-1)->pose.position.y-0.0006*(pathMsg.poses.end()-1)->pose.position.z - 0.2499;
    double newz=0.0080*(pathMsg.poses.end()-1)->pose.position.x	-0.0006*	(pathMsg.poses.end()-1)->pose.position.y+0.9999*(pathMsg.poses.end()-1)->pose.position.z+0.0092  ;
    this_pose_stamped.pose.position.x = newx;
    this_pose_stamped.pose.position.y = newy;
    this_pose_stamped.pose.position.z = newz;
    this_pose_stamped.pose.orientation.w=afterRotationQuat.w();
    this_pose_stamped.pose.orientation.x=afterRotationQuat.x();
    this_pose_stamped.pose.orientation.y=afterRotationQuat.y();
    this_pose_stamped.pose.orientation.z=afterRotationQuat.z();
    return this_pose_stamped;
}

geometry_msgs::Pose CloudBase::transformLiosamPathnew(const nav_msgs::Odometry::ConstPtr pathMsg){
    // [[ 0.99968442 -0.02004835 -0.01513714]
    //  [ 0.01981344  0.99968335 -0.01551225]
    //  [ 0.01544334  0.01520744  0.99976509]]
    // [ 1.65950261 -4.2548306   0.0313965 ]
    geometry_msgs::Pose this_pose_stamped;
    this_pose_stamped=pathMsg->pose.pose;

    Eigen::Matrix3d rotation_matrix;
    //0510 bag
    //     rotation_matrix<<  0.99968442, -0.02004835, -0.01513714,
    //  0.01981344 , 0.99968335 ,-0.01551225,
    //  0.01544334 , 0.01520744 , 0.99976509;
    //0524 bag
    //      rotation_matrix<<  -0.98172259, -0.19005012, 0.01008524 ,
    // 0.19022147,  -0.98153186,  0.02027313,
    //  0.00604607,  0.02182101,  0.99974361;
     rotation_matrix<< -0.9817312   ,0.19017827,  0.00600681,
    -0.19000758 ,-0.98153977,  0.02183553,
    0.01004857  ,0.02029529 , 0.99974353;
    Eigen::Quaterniond quaternion(rotation_matrix);
    Eigen::Quaterniond quaternionLIO(this_pose_stamped.orientation.w,this_pose_stamped.orientation.x,this_pose_stamped.orientation.y,this_pose_stamped.orientation.z);
    Eigen::Quaterniond afterRotationQuat=quaternion*quaternionLIO;
    //0510 bag
    // double newx= 0.99968442*pathMsg->pose.pose.position.x - 0.02004835*	pathMsg->pose.pose.position.y-0.01513714*pathMsg->pose.pose.position.z +1.65950261  ;
    // double newy=0.01981344*pathMsg->pose.pose.position.x+0.99968335*	pathMsg->pose.pose.position.y-0.01551225*pathMsg->pose.pose.position.z -4.2548306;
    // double newz=0.01544334*pathMsg->pose.pose.position.x	+0.01520744*	pathMsg->pose.pose.position.y+0.99976509*pathMsg->pose.pose.position.z+0.0313965  ;
    
    // double newx= -0.98172259*pathMsg->pose.pose.position.x - 0.19005012*	pathMsg->pose.pose.position.y+0.01008524*pathMsg->pose.pose.position.z -1.63281946  ;
    // double newy=0.19022147*pathMsg->pose.pose.position.x-0.98153186*	pathMsg->pose.pose.position.y+0.02027313*pathMsg->pose.pose.position.z-2.82577157;
    // double newz=0.00667484*pathMsg->pose.pose.position.x	+0.02158574*	pathMsg->pose.pose.position.y+0.99974361*pathMsg->pose.pose.position.z+0.01320576  ;

    double newx=  -0.9817312*pathMsg->pose.pose.position.x +0.19017827*	pathMsg->pose.pose.position.y+ 0.00600681*pathMsg->pose.pose.position.z -1.06462496  ;
    double newy=-0.19000758*pathMsg->pose.pose.position.x-0.98153977*	pathMsg->pose.pose.position.y+0.02183553*pathMsg->pose.pose.position.z-3.08371366;
    double newz=0.01004857*pathMsg->pose.pose.position.x	+0.02029529*	pathMsg->pose.pose.position.y+ 0.99974353*pathMsg->pose.pose.position.z+0.06008223  ;

    this_pose_stamped.position.x = newx;
    this_pose_stamped.position.y = newy;
    this_pose_stamped.position.z = newz;
    this_pose_stamped.orientation.w=afterRotationQuat.w();
    this_pose_stamped.orientation.x=afterRotationQuat.x();
    this_pose_stamped.orientation.y=afterRotationQuat.y();
    this_pose_stamped.orientation.z=afterRotationQuat.z();
    return this_pose_stamped;
}

//map and bag are not recorded in the same time, therefore bag's lio sam path has a transform with line map
void CloudBase::liosamPathCB (const nav_msgs::Path& pathMsg){
    // geometry_msgs::PoseStamped this_pose_stamped=transformLiosamPath(pathMsg);
    geometry_msgs::PoseStamped this_pose_stamped=*(pathMsg.poses.end()-1);
    TransformedLiosamPath.poses.push_back(this_pose_stamped);
    pubTransformedLiosamPath.publish(TransformedLiosamPath);
}

// record the AG index
void CloudBase::AGindexCB(const areaGraphDataParser::AGindex& msg){
    AG_index=msg;
    AGindexReceived=true;
}

// record the GT from liosam
void CloudBase::getGTfromLiosam(std_msgs::Header cloudHeader){
    tf::Quaternion quat;
    tf::quaternionMsgToTF((TransformedLiosamPath.poses.end()-1)->pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    GTstream << cloudHeader.stamp.toSec() << "\t"<<(TransformedLiosamPath.poses.end()-1)->pose.position.x << "\t"<<
    (TransformedLiosamPath.poses.end()-1)->pose.position.y << "\t"<<yaw/M_PI*180
        << endl;
}

void CloudBase::mapCB (const sensor_msgs::PointCloudConstPtr& laserCloudMsg){
    double mapCenterWeight=0;
    mapCenterInitialization<<0,0;
    mapInit=false;
    mapReceivedTimes++;
    //map change, histogram also need to change
    if(mapHistogram.size()!=0){
        mapHistogram.clear();
    }
    if(!mapInit){
        map_pc->clear();
        mapSize = laserCloudMsg->points.size(); 
        // map_pc->points.resize(mapSize+1);
        map_pc->points.resize(mapSize);

        // 遍历当前帧激光点云
        for (int i = 0; i < mapSize; i++)
        {
            // pcl格式
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserCloudMsg->points[i].x;
            thisPoint.y = laserCloudMsg->points[i].y;
            thisPoint.z = laserCloudMsg->points[i].z;
            thisPoint.intensity = 0;
            map_pc->points[i] = thisPoint;
            mapHistogram.push_back(0);

            double middile_x=(map_pc->points[i].x-map_pc->points[(i-1+mapSize)%mapSize].x)/2;
            double middile_y=(map_pc->points[i].y-map_pc->points[(i-1+mapSize)%mapSize].y)/2;
            double length=sqrt((map_pc->points[i].x-map_pc->points[(i-1+mapSize)%mapSize].x)*(map_pc->points[i].x-map_pc->points[(i-1+mapSize)%mapSize].x)+(map_pc->points[i].y-map_pc->points[(i-1+mapSize)%mapSize].y)*(map_pc->points[i].y-map_pc->points[(i-1+mapSize)%mapSize].y));
            mapCenterWeight+=length;
            mapCenterInitialization(0)+=middile_x*length;
            mapCenterInitialization(1)+=middile_y*length;
        }
        mapCenterInitialization(0)=mapCenterInitialization(0)/mapCenterWeight;
        mapCenterInitialization(1)=mapCenterInitialization(1)/mapCenterWeight;

        std::cout<<"map center = "<<mapCenterInitialization(0)<<","<<mapCenterInitialization(1)<<std::endl;
        // map_pc->points[mapSize].x=mapCenterInitialization(0);
        // map_pc->points[mapSize].y=mapCenterInitialization(1);
        // map_pc->points[mapSize].z=0;
        mapInit=true;
        ROS_WARN("cloudHandler Map initialized success, this is the %d map.", mapReceivedTimes);
    }
    pubPclCloud( map_pc, &pubMapPC, & mapHeader );
}

// get from data parse, use intensity to part different area
// start of a area, intensity=0;
// end of a area, intensity =2;
// normal node, intensity=1;
void CloudBase::mapAGCB (const sensor_msgs::PointCloudConstPtr& laserCloudMsg){
    ROS_WARN("receiveing map from AG");
    mapSize=map_pc->points.size();
    //receive index first, if already received, return
    //TODO: if the map changed....map need to be updated... --- 这是对应于论文中提到的换楼层map变化 -- AGmap Reload的情况
    
    // 如果还没收到AGindex，或者map已经初始化了，就返回，先不做CB处理
    if(!AGindexReceived || mapInit){
        return;
    }

    //handle map transform, transform map to GT FRAME
    std_msgs::Header tempHeader = laserCloudMsg->header;
    tempHeader.frame_id = "/map";

    // 转换点云格式：ROS PointCloud -> ROS PointCloud2
    sensor_msgs::PointCloud2 AGpointcloud2;
    // convertPointCloudToPointCloud2 这个函数定义在 sensor_msgs/point_cloud_conversion.h，通过utility.h引入
    // TODO: 这里的用意是: 原来的delin版本的map_pc是pointcloud格式，但是现在的map_pc已经发布为pointcloud2格式，因此这里的操作现在看来是多余的
    convertPointCloudToPointCloud2(*laserCloudMsg, AGpointcloud2);

    // 转换为PCL格式
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMap(new pcl::PointCloud<pcl::PointXYZI>());
    //Fujing: may be not necessary, just use map_pc --- Jiajie: 这里的意思是或许可以不用创建新的变量，直接在原来的map_pc上做变换
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserAGMapTansformed(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::moveFromROSMsg(AGpointcloud2, *laserAGMap);
   
    //构建变换矩阵
    Eigen::Matrix4f mapPose;
    mapPose.setZero();
    Eigen::Affine3f transform_initial = Eigen::Affine3f::Identity();

    // 设置平移和旋转yaw（参数在params.yaml）
    transform_initial.translation() << mapExtTrans[0], mapExtTrans[1], mapExtTrans[2];
    transform_initial.rotate (Eigen::AngleAxisf (mapYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ()));
    mapPose=transform_initial.matrix();

    // 应用变换
    pcl::transformPointCloud(*laserAGMap,*laserAGMapTansformed,mapPose);
    //publish to particle generator, need to generate particles with knowledge of which area the point is in
    // 粒子生成器要知道它需要在哪个区域中生成点
    pubPclCloud(laserAGMapTansformed, &pubAGMapTransformedPC, & tempHeader);

    double mapCenterWeight=0;
    mapCenterInitialization<<0, 0;
    mapReceivedTimes++;
    if(mapHistogram.size()!=0){
        mapHistogram.clear();
    }

    // 地图初始化
    if(!mapInit){
        map_pc->clear();
        mapSize = laserAGMapTansformed->points.size(); 
        map_pc->points.resize(mapSize);
        // 遍历所有map_pc
        for (int i = 0; i < mapSize; i++){
            // pcl格式  
            pcl::PointXYZI thisPoint;
            thisPoint.x = laserAGMapTansformed->points[i].x;  
            thisPoint.y = laserAGMapTansformed->points[i].y;
            thisPoint.z = laserAGMapTansformed->points[i].z;
            thisPoint.intensity =laserAGMapTansformed->points[i].intensity;
            // 用经过变换的点云替换原来的点云
            map_pc->points[i] = thisPoint;

            // 初始化直方图
            mapHistogram.push_back(0);
            
            // 计算地图中心 --- 加权平均法
            double middile_x=(map_pc->points[i].x-map_pc->points[(i-1+mapSize)%mapSize].x)/2;
            double middile_y=(map_pc->points[i].y-map_pc->points[(i-1+mapSize)%mapSize].y)/2;
            // 计算线段长度
            double length=sqrt((map_pc->points[i].x-map_pc->points[(i-1+mapSize)%mapSize].x)*(map_pc->points[i].x-map_pc->points[(i-1+mapSize)%mapSize].x)+(map_pc->points[i].y-map_pc->points[(i-1+mapSize)%mapSize].y)*(map_pc->points[i].y-map_pc->points[(i-1+mapSize)%mapSize].y));
            mapCenterWeight+=length;
            mapCenterInitialization(0)+=middile_x*length;
            mapCenterInitialization(1)+=middile_y*length;
        }

        // 计算最终的地图中心点 -- deprecated
        mapCenterInitialization /= mapCenterWeight;

        std::cout<<"map center = "<<mapCenterInitialization(0)<<","<<mapCenterInitialization(1)<<std::endl;
        mapInit=true;
        ROS_WARN("AG Map initialized success, this is the %d map.", mapReceivedTimes);

    }
    // 发布经过变换后的地图
    pubPclCloud(map_pc, &pubMapPC, &tempHeader);
}

void CloudBase::formFurthestRing(){
}

// not using - deprecated
void CloudBase::initializedUsingMassCenter(){
    double center_x=0;
    double center_y=0;
    for(int i =0;i<transformed_pc->points.size();i++){
        center_x+=transformed_pc->points[i].x;
        center_y+=transformed_pc->points[i].y;
    }
    center_x=center_x/transformed_pc->points.size();
    center_y=center_y/transformed_pc->points.size();
    std::cout<<"transformed PC CENTER = "<<center_x<<","<<center_y<<std::endl;
    robotPose(0,3)-=center_x-mapCenterInitialization(0);
    robotPose(1,3)-=center_y-mapCenterInitialization(1);
    // pcl::transformPointCloud(*organizedCloudIn,*transformed_pc,robotPose);
}

// 虚函数，被子类cloudHandler实现
void CloudBase::calClosestMapPoint(int inside_index){
    cout<<"CloudBase calClosestMapPoint, doing nothing"<<endl;
}

// filter invalid points, downsample
// 1d array N_SCAN*Horizon_SCAN
// get the furthest point of each column
// 函数作用：将原始的无序激光点云数据组织成有序的格式（按行列排列）， 过滤掉无效和不需要的点（如地面点、太远或太近的点）
// 输入： LaserCloudIn
// 输出 --- 函数计算后的结果存储在：
    // 1. organizedCloudIn64 -- 完整的64线点云
    // 2. organizedCloudIn -- 筛选后的有序点云(根据运行模式不同而存储不同内容：降采样点云或最远点)
void CloudBase::organizePointcloud(){
    // 初始化64线点云数组
    organizedCloudIn64->resize(64*Horizon_SCAN);

    // 如果是最远点跟踪模式 -- 全局定位模式，则设置为64线
    if(bFurthestRingTracking){
        N_SCAN=64;
    }
    int cloudSize = laserCloudIn->points.size(); 
    // furthestRing->clear();
    
    // 初始化最远点数组，每个水平角度保存一个最远点 
    furthestRing->points.resize(Horizon_SCAN,0);
    // transformedFurthestRing->clear();
    transformedFurthestRing->points.resize(Horizon_SCAN,0);

    // 主循环：对当前帧点云处理
    for (int i = 0; i < cloudSize; ++i)
    {
        // 转换为PCL格式点
        pcl::PointXYZI thisPoint;
        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        // TODO range_xy和range的意义是什么？range_xy是2D范围，range是3D范围
        // 计算水平距离，用作intensity
        float range_xy=sqrt(thisPoint.x*thisPoint.x+thisPoint.y*thisPoint.y);
        thisPoint.intensity =range_xy;
        float range = sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y + thisPoint.z*thisPoint.z);
        // check ring（获取激光号 --- 0~63）
        int rowIdn = laserCloudIn->points[i].ring;
        // 检查激光号是否合法
        if (rowIdn < 0 || rowIdn >= N_SCAN){
            ROS_ERROR("WRONG ROW_ID_N,CHECK N_SCAN....");
            continue;
        }
        // 线号降采样
        if (rowIdn % downsampleRate != 0){
            continue;
        }
        // 计算水平角度和列索引
        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;//与y轴夹角，[-pi,pi]
        // 水平扫描角度步长，例如一周扫描1800次，则两次扫描间隔角度0.2°
        static float ang_res_x = 360.0/float(Horizon_SCAN);
        int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;

        // 处理角度溢出
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN){
            continue;
        }
        //TODO: -0.8, RULE OUT GROUND POINTS
        // 计算一维索引
        int index = columnIdn + rowIdn * Horizon_SCAN;
        
        // 过滤无效点（如地面点、太远或太近的点） -- TODO: 为什么把无效点设置为0后仍然要存储？
        if (range < lidarMinRange || range > lidarMaxRange|| 
            laserCloudIn->points[i].z<groundThred || 
            laserCloudIn->points[i].z>ceilingThred) {

            // 将无效点设置为0
            thisPoint.x = thisPoint.y = thisPoint.z = thisPoint.intensity = 0;

            // 根据不同的运作模式存储点云
            if(!initialized && bRescueRobot){
                organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
            }
            else{
                organizedCloudIn->points[index] = thisPoint;
                organizedCloudIn64->points[index] = thisPoint;
            }
            continue;
        }

        if((!initialized && bRescueRobot)){
            organizedCloudIn->points[index%Horizon_SCAN] = thisPoint;
        }
        //only localization
        else{
            organizedCloudIn->points[index] = thisPoint;
            organizedCloudIn64->points[index] = thisPoint;
        }
        // save the furthest point of each column for clutter free point set
        if(range_xy > furthestRing->points[columnIdn].intensity){
            furthestRing->points[columnIdn]=thisPoint;
            furthestRing->points[columnIdn].intensity=range_xy;
        }
    }
    // rescue without initialized
    if(!initialized&&bRescueRobot){
        *organizedCloudIn=*furthestRing;
    }
    if(bFurthestRingTracking){
        *organizedCloudIn=*furthestRing;
    }
    // pubPclCloud( furthestRing, &pubFurthestRing, & cloudHeader );
    if(bFurthestRingTracking){
        N_SCAN=1;
    }
}

//set robot pose from params.yaml
void CloudBase::setInitialPose(double initialYawAngle, Eigen::Vector3f initialExtTrans){
    robotPose.setZero();
    Eigen::Affine3f transform_initial = Eigen::Affine3f::Identity();
    transform_initial.translation() << initialExtTrans[0], initialExtTrans[1], initialExtTrans[2];
     // rotate around Z axis
    transform_initial.rotate (Eigen::AngleAxisf (initialYawAngle/180.0*M_PI, Eigen::Vector3f::UnitZ()));
    // cout<<"initialYAW angle ="<<initialYawAngle<<endl;
    // Eigen::AngleAxisf rotation_vector2;
    // rotation_vector2.fromRotationMatrix(initialExtRot);
    // transform_initial.rotate (rotation_vector2);
    robotPose=transform_initial.matrix();
    // cout<<"robotPose angle="<<robotPose<<endl;
}

//allocate memory for pcl ptr
void CloudBase::allocateMemory(){
   cout<<"base allocate memory, doing nothing"<<endl;
}

//called every frame and every guess
void CloudBase::resetParameters(){
    cout<<"base resetParameters, doing nothing"<<endl;
}

void CloudBase::setEveryFrame(){
    bPCA=false;
    accumulateAngle=0;
    averDistancePairedPoints=0;
    onlyOneDirection         = false;
    //mapHistogram has same size as map, calculate every frame of lidar data
    std::fill(mapHistogram.begin(), mapHistogram.end(), 0);
    numTotalHistogram=0;
}

void CloudBase::pubPclCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud , ros::Publisher * pub, std_msgs::Header* cloudHeader ){
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*cloud, tempCloud);
    tempCloud.header=*cloudHeader;
    pub->publish(tempCloud);
}
