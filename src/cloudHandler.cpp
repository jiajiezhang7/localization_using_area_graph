#include "utility.h"
#include"cloudHandler.hpp"

// the function to get corridorness downsample rate
double CloudHandler::corridornessDSRate(double maxPercentage){
    if(maxPercentage<0.5){
        return 0;
    }
    else{
        return 10*maxPercentage-4;
    }
}

// check robot is now inside which area, a smarter way is to get connected area based on passage
void CloudHandler::gettingInsideWhichArea(){

    pcl::PointCloud<pcl::PointXYZI>::Ptr insideAreaPC(new pcl::PointCloud<pcl::PointXYZI>);

    // 如果有上一次的区域的index,那么判断是否还在上一次的区域内
    if(lastInsideIndex!=-1){
        bool binside;
        // 检查是否还在上一次的区域
        binside=areaInsideChecking(robotPose, lastInsideIndex);
        // 如果还在上一次的区域内
        if(binside){
            // 收集该区域的所有地图点node，用于可视化
            for(int j=lastInsideIndex;j<lastInsideIndex+100000;j++){
                // intensity % 3 == 2 means end of an area
                if((int)map_pc->points[j].intensity%3==2){
                    break;
                }
                insideAreaPC->points.push_back(map_pc->points[j]);
            }
        // 发布可视化信息并返回
        pubPclCloud(insideAreaPC,&pubinsideAreaPC,&mapHeader);
        cout<<"----------------------- inside old area -----------------------"<<endl;
        return;
        }
    }

    // 如果不在上一次的区域内，那么重新计算
    int insideTime=0;      // 计算机器人在几个区域内
    int temp=-1;           // 区域ID计数器

    for(int i =0;i<map_pc->points.size();i++){
        bool binside=false;

        //intensity % 3 == 0 means start of an area
        if((int)map_pc->points[i].intensity%3==0){
            binside=areaInsideChecking(robotPose, i);
            temp++;
        }
        // 如果找到包含机器人的Area
        if(binside){
            insideTime++;              //记录找到的可能的Area数
            insideAreaStartIndex=i;    //记录找到的可能的Area的起始index
            insideAreaID=temp;         // 记录找到的可能的Area的ID
            // 收集该区域的所有地图点node，用于可视化
            for(int j=i;j<i+100000;j++){
                if((int)map_pc->points[j].intensity%3==2) break;
                insideAreaPC->points.push_back(map_pc->points[j]);
            }
            lastInsideIndex=i;   // 更新Area索引，用于下一次判断
        }
    }
    // map_pc 只包含叶子区域，机器人应该只在一个区域内
    // 异常处理：
    if(insideTime>1){
        ROS_ERROR("ERROR, ROBOT POSE INSIDE SEVERAL AREAS!!!!!!!!!!!!!!");
    }else if(insideTime==0){
        ROS_ERROR("ERROR, ROBOT POSE OUTSIDE ALL AREAS!!!!!!!!!!!!!!");
    }
    else{
        ROS_INFO("ROBOT POSE INSIDE AREA %d",insideAreaStartIndex);
    }
    cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
    // visualization
    pubPclCloud(insideAreaPC,&pubinsideAreaPC,&mapHeader);
    if(insideTime>1){
        ROS_ERROR("ERROR, ROBOT POSE INSIDE SEVERAL AREAS!!!!!!!!!!!!!!");
        while(1){
            std::cout<<"press enter to continue "<<std::endl;
            std::getchar();
            break;
        } 
    }
}

CloudHandler::CloudHandler(){

    globalImgTimes = 0;        // 全局图像计数器
    getGuessOnce = false;      // 是否收到全局定位的初始猜测
    sumFrameRunTime = 0;       // 累计运行时间
    numofFrame = 0;            // 帧计数器

    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &CloudHandler::cloudHandlerCB, this, ros::TransportHints().tcpNoDelay());
    pubinsideAreaPC= nh.advertise<sensor_msgs::PointCloud2> ("insideAreaPC", 1);
    subInitialGuess    = nh.subscribe<sensor_msgs::PointCloud>("/particles_for_init",1, &CloudHandler::getInitialExtGuess, this,ros::TransportHints().tcpNoDelay());

    // 记录机器人位姿结果
    robotPoseTum.open("/home/xiefujing/research/area_graph/ws/robotPoseResult/robotPoseTum.txt", ofstream::ate);
    robotPoseTum.setf(ios::fixed);
    robotPoseTum.precision(6);

    // 记录LIO-SAM位姿结果
    LiosamPoseTum.open("/home/xiefujing/research/area_graph/ws/robotPoseResult/LiosamPoseTum.txt", ofstream::ate);
    LiosamPoseTum.setf(ios::fixed);
    LiosamPoseTum.precision(6);
    // subscribe liosam GT, use evo -a align result, no need to change to same frame
    // use /lio_sam/mapping/odometry for 2023-05-10-20-16-52.bag
    // subLiosamPath=nh.subscribe("/lio_sam/mapping/odometry_incremental",1000,&CloudHandler::liosamOdometryIncrementalCB, this, ros::TransportHints().tcpNoDelay());
    subLiosamPath=nh.subscribe("/lio_sam/mapping/odometry",1000,&CloudHandler::liosamOdometryIncrementalCB, this, ros::TransportHints().tcpNoDelay());
    allocateMemory();
    // cloudInitializer(nh);
    ROS_WARN(" CLOUD HANDLER READY");
    //out of param.yaml
    // 设置初始位姿
    setInitialPose(initialYawAngle,initialExtTrans);
}

void CloudHandler::getInitialExtGuess(const sensor_msgs::PointCloudConstPtr& laserCloudMsg){
    getGuessOnce=true;
}

void CloudHandler::liosamOdometryIncrementalCB(const nav_msgs::Odometry::ConstPtr& odomMsg){
    LiosamPoseTum<<odomMsg->header.stamp.toSec()<<" "<<odomMsg->pose.pose.position.x<<" "<<odomMsg->pose.pose.position.y<<" "<<odomMsg->pose.pose.position.z<<" "<<odomMsg->pose.pose.orientation.x<<" "<<odomMsg->pose.pose.orientation.y<<" "<<odomMsg->pose.pose.orientation.z<<" "<<odomMsg->pose.pose.orientation.w<<std::endl;
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header=odomMsg->header;
    // this_pose_stamped.pose=odomMsg->pose.pose;
    this_pose_stamped.pose=transformLiosamPathnew(odomMsg);
    TransformedLiosamPath.poses.push_back(this_pose_stamped);
    pubTransformedLiosamPath.publish(TransformedLiosamPath);
}

void CloudHandler::cloudHandlerCB(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    // N_SCAN=64;
    if(globalImgTimes==0){
        string words="Global localizing";
        showImg1line(words);
    }
    globalImgTimes++;


    clock_t startTime,endTime,startTimecb;
    auto startC = std::chrono::high_resolution_clock::now();
    startTime = ros::Time::now().toNSec();
    startTimecb = ros::Time::now().toNSec();
    //shoot through passage map_pc index record
    outsideAreaIndexRecord.clear(); 
    outsideAreaLastRingIndexRecord.clear(); 


    // 检查地图是否初始化
    if(!mapInit){
        ROS_WARN("Map not initialized yet, waiting for map!!!!!!!!!!");
        return;
    }

    setEveryFrame();
    cloudInitializer.setMapPC(map_pc);
    cloudHeader=laserCloudMsg->header;
    mapHeader=cloudHeader;
    mapHeader.frame_id="/map";
    globalPath.header=mapHeader;

    // getGTfromLiosam(cloudHeader);

    // 重点：转换点云格式并组织
    sensor_msgs::PointCloud2 temp_msg=*laserCloudMsg;
    //ros pointcloud to pcl xyzirt
    pcl::moveFromROSMsg(temp_msg, *laserCloudIn);
    organizePointcloud();


    if(bFurthestRingTracking){
        // N_SCAN=1;
    }
    pubPclCloud( organizedCloudIn, &pubOrganizedCloudIn, & cloudHeader );
    //transform pointcloud robotPose T_wl

    //normal localization 
    if(initialized){
        errorUpThredCurr = errorUpThred;
        errorLowThredCurr=errorLowThred;
    }
    else{
        errorUpThredCurr = errorUpThredInit;
        errorLowThredCurr=errorLowThredInit;
    }

    if(bTestRescue){
        // 测试全局定位效果的模式：每帧都进行全局定位，不保存历史位姿。主要用于调试和验证全局定位算法的性能
        ROS_WARN("TEST RESCUE ROBOT, EVERY FRAME GOES TO RESCUE");
        // CloudInitializer cloudInitializer(furthestRing,map_pc,mapHeader,nh);
        cloudInitializer.nh=nh;

        // 订阅估计的全局位姿（这也是WiFi-Loc的接口）
        cloudInitializer.subInitialGuess  = nh.subscribe<sensor_msgs::PointCloud>("/particles_for_init",1, &CloudInitializer::getInitialExtGuess, &cloudInitializer,ros::TransportHints().tcpNoDelay());
        // 发布最远点云用于可视化
        // TODO 最远点云furthestRing从哪里来
        pubPclCloud( furthestRing, &pubtest, & mapHeader );
        // 设置激光点云数据
        cloudInitializer.setLaserCloudin(furthestRing,mapHeader);
        resetParameters();

        return;
    }
    else if(bRescueRobot){
        // 全局定位+位姿跟踪模式：只在需要时进行一次全局定位，，获得位姿后就切换回正常模式
        if(!getGuessOnce) return; 
        ROS_WARN("ONLY ONE FRAME GOES TO RESUCUE ROBOT");
        cloudInitializer.nh=nh;
        cloudInitializer.subInitialGuess    = nh.subscribe<sensor_msgs::PointCloud>("/particles_for_init",1, &CloudInitializer::getInitialExtGuess, &cloudInitializer,ros::TransportHints().tcpNoDelay());
        // cloudInitializer.subInitialGuess    = nh.subscribe<sensor_msgs::PointCloud>("/particles_for_init",1, boost::bind(&CloudInitializer::getInitialExtGuess,_1,robotPose), &cloudInitializer,ros::TransportHints().tcpNoDelay());

        pubPclCloud( furthestRing, &pubtest, & mapHeader );
        
        cloudInitializer.setLaserCloudin(furthestRing,mapHeader);
        resetParameters();

        // 确定机器人位姿（得分最高的那个）
        robotPose=cloudInitializer.MaxRobotPose;
        cout<<"setting robot pose in rescue robot, "<<robotPose(0,3)<<","<<robotPose(1,3)<<endl;

        // 完成全局定位，切换回正常模式（下次CB的时候就会进入下一个else）
        bRescueRobot=false;
        
        // 取消订阅初始粗略位姿估计话题 
        // TODO 这里为什么不写成cloudInitializer.subInitialGuess?
        subInitialGuess  = nh.subscribe<sensor_msgs::PointCloud>("/none",1, &CloudHandler::getInitialExtGuess, this,ros::TransportHints().tcpNoDelay());
        return;
    }
    else{
        // 只进行位姿跟踪模式（全局定位由params.yaml给出）
        // TODO getGuessOnce是什么意思？
        if(getGuessOnce){
            robotPose=cloudInitializer.MaxRobotPose;
            errorUpThred=3;   // 设置误差阈值
            // 取消订阅初始粗略位姿估计话题
            cloudInitializer.subInitialGuess  = nh.subscribe<sensor_msgs::PointCloud>("/none",1, &CloudInitializer::getInitialExtGuess, &cloudInitializer,ros::TransportHints().tcpNoDelay());
            ROS_ERROR("SETTING ERRORUPTHRED=3");
            // cloudInitializer.~CloudInitializer();
            getGuessOnce=false;
            string words="Pose tracking";
            showImg1line(words);
        }
        // localization pipeline
        ROS_WARN("NO FRAME GOES TO RESCUE, USE EXT MAT IN PARAM.YAML");

        // 转换点云到机器人坐标系
        pcl::transformPointCloud(*organizedCloudIn,*transformed_pc,robotPose);
        cout<<" robot pose in tracking , "<<robotPose(0,3)<<","<<robotPose(1,3)<<endl;

        pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader);
        endTime = ros::Time::now().toNSec();
        cout << "The prepare run time is:" <<( endTime - startTime) / 1e6 << "ms" << endl;


        vbHistogramRemain.resize(transformed_pc->points.size(),true);

        startTime = ros::Time::now().toNSec();
        // 定位主要步骤：1. 确定机器人在哪个区域内
        gettingInsideWhichArea();
        endTime = ros::Time::now().toNSec();
        cout << "The gettingInsideWhichArea run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;


        startTime = ros::Time::now().toNSec();
        // 定位主要步骤：2. 计算最近的地图点
        calClosestMapPoint(insideAreaStartIndex);
        endTime = ros::Time::now().toNSec();
        cout << "The calClosestMapPoint run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;

        startTime = ros::Time::now().toNSec();
        // 定位主要步骤：3. 进行ICP优化，更新机器人位姿
        optimizationICP();

        // 发布经过ICP优化后的点云，用于观察定位优化结果
        pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader);


        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc_(new pcl::PointCloud<pcl::PointXYZI> );
        // 使用优化后的机器人位姿，转换完整的原始点云并发布
        transformed_pc_->resize(64*Horizon_SCAN);

        pcl::transformPointCloud(*organizedCloudIn64,*transformed_pc_,robotPose);
        pubPclCloud( transformed_pc_, &pubTransformedWholePC, & mapHeader );
        transformed_pc_->clear();

        // 使用优化后的机器人位姿，转换最远点云并发布
        transformed_pc_->resize(Horizon_SCAN);
        pcl::transformPointCloud(*furthestRing,*transformed_pc_,robotPose);
        pubPclCloud( transformed_pc_, &pubFurthestRing, & cloudHeader);
        endTime = ros::Time::now().toNSec();
        cout << "The optimizationICP run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;

    }
        // pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
        //initializedUsingMassCenter();
    resetParameters();
    // initialized=true;
    endTime = ros::Time::now().toNSec();
    auto finishC = std::chrono::high_resolution_clock::now();
    cout << "The pointcloud_CB run time is:" << (double)(endTime - startTimecb) / 1e6 << "ms" << endl;
    if( (double)(endTime - startTimecb) / 1e6>100){
        ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!TAKES TOO LONG");
        // while(1){
        //     std::cout<<"press enter to continue "<<std::endl;
        //     std::getchar();
        //     break;
        // } 
    }
    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(finishC-startC).count() << "ns\n";
    sumFrameRunTime+=std::chrono::duration_cast<std::chrono::nanoseconds>(finishC-startC).count();
    numofFrame++;
    std::cout << "Average cloudhandler run time is " <<sumFrameRunTime/numofFrame<< "ns\n"<<std::endl;
}

// 用于计算激光点云与地图的交点，为后续的ICP匹配做准备
// inside_index - 计算自gettingInsideWhichArea: 机器人所在Area的起始点index
void CloudHandler::calClosestMapPoint(int inside_index){
    int last_index=0;
    // 遍历每个水平角度
    for(int i=0;i<Horizon_SCAN;i++){ // Horizon_SCAN是水平方向的点数
        bool findIntersection=false;
        // 最远点跟踪模式
        if(bFurthestRingTracking){
            double minDist;
            // 只检查第0线 --- 最远点 与地图的交点
            findIntersection=checkMap(0,i,last_index,minDist,inside_index);
        }
        else{
            // 每隔5线检查一次，从10线开始
            for(int chose_ring=0; chose_ring<N_SCAN/5;chose_ring++){
                double minDist;
                if((10+5*chose_ring)<N_SCAN){
                    // 检查第10,15,20...线的交点
                    findIntersection=checkMap(10+5*chose_ring,i,last_index,minDist,inside_index);
                }
                if(findIntersection) break; // 找到交点就停止
            }
        }
        // 如果没找到交点，则设置为0点
        if(!findIntersection){
            intersectionOnMap->points[i].x=0;
            intersectionOnMap->points[i].y=0;
            intersectionOnMap->points[i].z=0;
        }
    }
    pubPclCloud(intersectionOnMap, &pubIntersection, & mapHeader );
}

// visualization in rviz
void CloudHandler:: showImg1line(string words){
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("Things2say", 1);
    cv::Mat image(200, 600, CV_8UC3, cv::Scalar(0,0,0));
    putText(image,words,cv::Point(20,100),cv::FONT_HERSHEY_DUPLEX,2,cv::Scalar(255,255,255),2,8);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pub.publish(msg);
    ROS_ERROR("THINGS TO SAY!!!!!!!!!!!!!!!!!!!!!! 1line %s",words);
}

// according to robot pose and map, save this ray should intersect with which map point, and its intersection.
bool CloudHandler::checkMap(int ring,int horizonIndex,int& last_index,double & minDist,int inside_index){
    pcl::PointXYZI PCPoint;
    PCPoint.x =transformed_pc->points[ring*Horizon_SCAN+horizonIndex].x;
    PCPoint.y =transformed_pc->points[ring*Horizon_SCAN+horizonIndex].y;
    PCPoint.z =0;
    pcl::PointXYZI PosePoint;
    PosePoint.x=robotPose(0,3);
    PosePoint.y=robotPose(1,3);
    PosePoint.z=0;
    bool findIntersection=false;
    minDist=0;
    //traverse map
    //BUG: last_index should not change in for
    for(int j=inside_index;j<mapSize;j++){
        // the last map point of a area
        if((int)map_pc->points[j%mapSize].intensity%3==2){
            break;
        }
        //means it is glass, defined in data parser
        if(map_pc->points[j%mapSize].z!=0){
            continue;
        }
        //WRONG map point..........
        //FIXME:
        // if((j%mapSize==28)||(j%mapSize==32)){
        //     continue;
        // }
        pcl::PointXYZI intersectionOnMapThisLine;
        bool inbetween=inBetween(PosePoint,PCPoint,map_pc->points[j%mapSize],map_pc->points[(j+1)%mapSize],&intersectionOnMapThisLine);
        if(inbetween){
            //ray may intersect with map several times since the shape of the map polygon is not guarteened to be convex, chose the closest intersection                                
            if(minDist==0 ||
                        minDist>((intersectionOnMapThisLine.x-PosePoint.x)*(intersectionOnMapThisLine.x-PosePoint.x)
                                            +(intersectionOnMapThisLine.y-PosePoint.y)*(intersectionOnMapThisLine.y-PosePoint.y))){
                findIntersection=true;
                minDist=(intersectionOnMapThisLine.x-PosePoint.x)*(intersectionOnMapThisLine.x-PosePoint.x)+(intersectionOnMapThisLine.y-PosePoint.y)*(intersectionOnMapThisLine.y-PosePoint.y);
                intersectionOnMap->points[horizonIndex]=intersectionOnMapThisLine;
                // passage line, defined in osm_ag_lib
                if((int)map_pc->points[j%mapSize].intensity>2&&(int)map_pc->points[(j+1)%mapSize].intensity>2){
                    // mark this point intersection with passage, make a difference when test all passage open
                    intersectionOnMap->points[horizonIndex].intensity=-1;
                }
                // mark this ray point is intersection with which map line point, used in pedal calculation
                ringMapP1->points[horizonIndex]=map_pc->points[j%mapSize];
                ringMapP2->points[horizonIndex]=map_pc->points[(j+1)%mapSize];
                last_index=j%mapSize;
                //case2 after rescue and case3 
                if(initialized||(!bTestRescue&&!bRescueRobot)){
                    for(int i=0;i<N_SCAN;i++){
                        transformed_pc->points[i*Horizon_SCAN+horizonIndex].intensity=j%mapSize;
                    }
                }
                else{
                    transformed_pc->points[horizonIndex].intensity=j%mapSize;
                }
            }
        }
        else{
            continue;
        }
    }
    return findIntersection;
}

// check whole map for intersection, since the robot could see through the passage and glass, this time find the intersection closest to lidar point, not the first one of ray tracing.
bool CloudHandler::checkWholeMap(int pc_index, const pcl::PointXYZI& PCPoint,double &map1x,double &map1y,double &map2x,double &map2y,double & intersectionx, double & intersectiony){
    pcl::PointXYZI PosePoint;
    PosePoint.x=robotPose(0,3);
    PosePoint.y=robotPose(1,3);
    PosePoint.z=0;

    double min_error=0;
    double min_PCLength=0;
    double min_mapLength=0;
    bool first_find=false;
    bool first_ring_find=false;
    bool done_checking_ring=false;
    int start_index=0;
    bool bMatchWithPass=false;

    if(outsideAreaIndexRecord[pc_index]!=0){
        start_index=outsideAreaIndexRecord[pc_index];
    }
    //distance between this ring and last ring is small , then probable match with the same map line, save computation
    else if(outsideAreaLastRingIndexRecord[pc_index%Horizon_SCAN]!=0
                &&calDistance(transformed_pc->points[pc_index-Horizon_SCAN],transformed_pc->points[pc_index])<0.8){
        start_index=outsideAreaLastRingIndexRecord[pc_index%Horizon_SCAN];
    }
    else{
        start_index=0;
    }
    for(int i=start_index;i<map_pc->size()+start_index;i++){
        // TODO:maybe not necessary, for each guess, same number of inside to be ignore?????
        // bool bOnRay=false;
        // inRay(PosePoint,PCPoint,map_pc->points[i%mapSize],map_pc->points[(i+1)%mapSize],bOnRay);
        // if(bOnRay&&(PCPoint.x!=0||PCPoint.y!=0)){
        //     numofIntersection[horizonIndex]+=1;
        // }
        if(bAllPassageOpen){
            //it is a passage map line
            if((int)map_pc->points[i%mapSize].intensity>2&&(int)map_pc->points[(i+1)%mapSize].intensity>2){
                continue;
            }
        }
        //since the end of a area should not be connected with the start of the next area.
        if(((int)map_pc->points[i%mapSize].intensity)%3==2){
            continue;
        }
        // if(map_pc->points[i%mapSize].z!=0){
        //     continue;
        // }
        pcl::PointXYZI intersectionOnMapThisLine;
        bool inbetween=inBetween(PosePoint,PCPoint,map_pc->points[i%mapSize],map_pc->points[(i+1)%mapSize],&intersectionOnMapThisLine);
        if(inbetween){
            //chose the closest intersection to actual point, not the first ray tracing intersection                               
            double dist=calDistance(intersectionOnMapThisLine,PCPoint);
            if(min_error==0 ||min_error>dist){
                // findIntersection=true;
                min_error=dist;
                map1x=map_pc->points[i%mapSize].x;
                map1y=map_pc->points[i%mapSize].y;
                map2x=map_pc->points[(i+1)%mapSize].x;
                map2y=map_pc->points[(i+1)%mapSize].y;
                // cout<<"size  = "<<intersectionOnMap->points.size()<<","<<ringMapP1->points.size()<<","<<ringMapP2->points.size()<<endl;
                // intersectionOnMap->points[horizonIndex]=intersectionOnMapThisLine;
                //mark if pc length is shorter than map length
                double mapLength=calDistance(intersectionOnMapThisLine,PosePoint);
                // double PCLength=calDistance(PCPoint,PosePoint);
                double PCLength=calDistance(PCPoint,PosePoint);

                min_mapLength=mapLength;
                min_PCLength=PCLength;
                outsideAreaIndexRecord[pc_index]=i%mapSize;
                outsideAreaLastRingIndexRecord[pc_index%Horizon_SCAN]=i%mapSize;
                intersectionx=intersectionOnMapThisLine.x;
                intersectiony=intersectionOnMapThisLine.y;
                if(map_pc->points[i%mapSize].intensity>2){
                    bMatchWithPass=true;
                }
            }
            //if not the first ring  of this frame then stop searching
            if(start_index){
                break;
            }
        }
        else{
            continue;
        }
    }
    //didn't find ideal intersection
    if(bMatchWithPass&&min_error>1){
        return true;
    }
    else{
        return false;
    }
}

void CloudHandler::filterUsefulPoints(){
        auto  startTime = ros::Time::now().toNSec();
        //reset center for each iteration
        PCCenter(0)=0;
        PCCenter(1)=0;
        PCCenter.setZero();
        mapCenter.setZero();
        numIcpPoints=0;
        usefulIndex.clear();
        weightSumTurkey=0;
        weightSumCauchy=0;
        weightsTurkey.clear();
        outsideAreaIndexRecord.resize(transformed_pc->points.size(),0);
        outsideAreaLastRingIndexRecord.resize(Horizon_SCAN,0);

        for(int i=0;i<transformed_pc->points.size();i++){
            if(std::isnan(transformed_pc->points[i].x)||std::isnan(transformed_pc->points[i].y)
                ||std::isnan(intersectionOnMap->points[i%(Horizon_SCAN)].x)||std::isnan(intersectionOnMap->points[i%(Horizon_SCAN)].y)){
                ROS_ERROR_ONCE("nan points in transformed pc!!!!!!!!");
                continue;
            }
            //distance from robot to lidarpoints, transformed_pc is in world frame
            // double distance=sqrt((transformed_pc->points[i].x-robotPose(0,3))*(transformed_pc->points[i].x-robotPose(0,3))+(transformed_pc->points[i].y-robotPose(1,3))*(transformed_pc->points[i].y-robotPose(1,3)));
            double distance=organizedCloudIn->points[i].intensity;
            if(abs(intersectionOnMap->points[i%(Horizon_SCAN)].x)>1e-6&&abs(intersectionOnMap->points[i%(Horizon_SCAN)].y)>1e-6){
                //calculare pedal
                double pedalx;
                double pedaly;
                double intersectionx;
                double intersectiony;
                // means this point is matching with passage
                // double dis2line=calDistance2Line(transformed_pc->points[i],ringMapP1->points[i%Horizon_SCAN],ringMapP2->points[i%Horizon_SCAN]);
                double temp_map_length=sqrt((intersectionOnMap->points[i%(Horizon_SCAN)].x-robotPose(0,3))*(intersectionOnMap->points[i%(Horizon_SCAN)].x-robotPose(0,3))+(intersectionOnMap->points[i%(Horizon_SCAN)].y-robotPose(1,3))*(intersectionOnMap->points[i%(Horizon_SCAN)].y-robotPose(1,3)));
                double match_difference=distance-temp_map_length;

                // abilation test for how to treat passage, it is always treated as opened or closed
                if(!bAllPassageClose&&!bAllPassageOpen){
                    // to save computation, only calculate the points that are far away from intersection, but this may cause mismatch during going through door, since many points in near place
                    // threshold may related to max velocity
                    // handle points hitting passage
                    if(match_difference>0.1&&ringMapP1->points[i%Horizon_SCAN].intensity>2&&ringMapP2->points[i%Horizon_SCAN].intensity>2){
                        double map1x=0;
                        double map1y=0;
                        double map2x=0;
                        double map2y=0;
                        startTime = ros::Time::now().toNSec();
                        bool countGoingthrough=
                        checkWholeMap(i,transformed_pc->points[i],map1x,map1y,map2x,map2y,intersectionx,intersectiony);
                        //special threshold for going through door, don't match with clutter through door.
                        //didn't find ideal intersection
                        if(countGoingthrough){
                            continue;
                        }
                        auto endTime = ros::Time::now().toNSec();
                            // cout << "--------------------The checkWholeMap run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
                        calPedal(map1x,map1y,map2x,map2y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
                    }
                    else{
                        calPedal(ringMapP1->points[i%Horizon_SCAN].x,ringMapP1->points[i%Horizon_SCAN].y,ringMapP2->points[i%Horizon_SCAN].x,ringMapP2->points[i%Horizon_SCAN].y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
                        intersectionx=intersectionOnMap->points[i%(Horizon_SCAN)].x;
                        intersectiony=intersectionOnMap->points[i%(Horizon_SCAN)].y;
                    }
                }
                else if(bAllPassageClose){
                    // no shoot through, only match inside current area
                    calPedal(ringMapP1->points[i%Horizon_SCAN].x,ringMapP1->points[i%Horizon_SCAN].y,ringMapP2->points[i%Horizon_SCAN].x,ringMapP2->points[i%Horizon_SCAN].y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
                    intersectionx=intersectionOnMap->points[i%(Horizon_SCAN)].x;
                    intersectiony=intersectionOnMap->points[i%(Horizon_SCAN)].y;
                }
                else if(bAllPassageOpen){
                    if(intersectionOnMap->points[i%(Horizon_SCAN)].intensity==-1){
                        // if not supposed to match with passage, check whole map
                        if(match_difference>0.5){
                            double map1x=0;
                            double map1y=0;
                            double map2x=0;
                            double map2y=0;
                            startTime = ros::Time::now().toNSec();
                            bool countGoingthrough=
                            checkWholeMap(i,transformed_pc->points[i],map1x,map1y,map2x,map2y,intersectionx,intersectiony);
                            //special threshold for going through door, don't match with clutter through door.
                            //didn't find ideal intersection
                            if(countGoingthrough){
                                continue;
                            }
                            calPedal(map1x,map1y,map2x,map2y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
                        }
                        else{
                            cout<<"fffffffff"<<match_difference<<", distance= "<<distance<<"temp_map_length="<<temp_map_length<<"intersectionOnMap->points[i%(Horizon_SCAN)].x"<<intersectionOnMap->points[i%(Horizon_SCAN)].x<<"robotPose(0,3)"<<robotPose(0,3)<<endl;
                            continue;
                        }

                    }
                    else{
                        calPedal(ringMapP1->points[i%Horizon_SCAN].x,ringMapP1->points[i%Horizon_SCAN].y,ringMapP2->points[i%Horizon_SCAN].x,ringMapP2->points[i%Horizon_SCAN].y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
                        intersectionx=intersectionOnMap->points[i%(Horizon_SCAN)].x;
                        intersectiony=intersectionOnMap->points[i%(Horizon_SCAN)].y;
                    }
                }


                // check if pedal point is on the middle of map line, if not, continue
                // if((pedalx>ringMapP1->points[i%Horizon_SCAN].x&&pedalx>ringMapP2->points[i%Horizon_SCAN].x)
                //     ||(pedalx<ringMapP1->points[i%Horizon_SCAN].x&&pedalx<ringMapP2->points[i%Horizon_SCAN].x)){
                //     continue;
                // }
                double pcx=transformed_pc->points[i].x;
                double pcy=transformed_pc->points[i].y;
                //intersectionx and intersectiony may be different than the map intersection due to through passage
                double maplength=sqrt((intersectionx-robotPose(0,3))*(intersectionx-robotPose(0,3))+(intersectiony-robotPose(1,3))*(intersectiony-robotPose(1,3)));
                double error=distance-maplength;
                double error_vertical=sqrt((pedalx-pcx)*(pedalx-pcx)+(pedaly-pcy)*(pedaly-pcy));
                //funiture
                // if(vbHistogramRemain[i]&&(error<0.0&&((-error_vertical)>(-errorLowThredCurr)))||
                // UsefulPoints are the point sets used in ICP, filter out points too far away
                if((error<0.0&&((error_vertical<errorLowThredCurr)))||
                    (error>0.0&&(error_vertical<errorUpThredCurr))){
                        numIcpPoints++;
                        usefulIndex.push_back(i);  
                        UsefulPoints1->points[i]=transformed_pc->points[i];
                        UsefulPoints2->points[i].x=pedalx;
                        UsefulPoints2->points[i].y=pedaly;
                        UsefulPoints2->points[i].z=transformed_pc->points[i].z;
                        //handle weight
                        double weight = calWeightTurkey(error_vertical,errorLowThredCurr,(error>0),errorUpThredCurr);
                        weightSumTurkey+=weight;
                        weightsTurkey.push_back(weight);
                        // when not initialized, use unweighted icp
                        if(use_weight&&initialized){
                            double tempx=weight*pcx;
                            double tempy=weight*pcy;
                            PCCenter(0)+=tempx;
                            PCCenter(1)+=tempy;
                            double tempx_=weight*pedalx;
                            double tempy_=weight*pedaly;
                            mapCenter(0)+=weight*pedalx;
                            mapCenter(1)+=weight*pedaly;

                            Vec_pcx.push_back(tempx);
                            Vec_pcy.push_back(tempy);
                            Vec_pedalx.push_back(tempx_);
                            Vec_pedaly.push_back(tempy_);
                        }
                        else{
                            PCCenter(0)+=pcx;
                            PCCenter(1)+=pcy;
                            mapCenter(0)+=pedalx;
                            mapCenter(1)+=pedaly;
                            Vec_pcx.push_back(pcx);
                            Vec_pcy.push_back(pcy);
                            Vec_pedalx.push_back(pedalx);
                            Vec_pedaly.push_back(pedaly);
                        }
                        averDistancePairedPoints+=error_vertical;
                        if(currentIteration==0){
                            mapHistogram[transformed_pc->points[i].intensity]++;
                            numTotalHistogram++;
                        }                       
                    }
                    //maybe furniture or clutter
                    else{
                        continue;
                    }
                }
                // //too far away from what supposed to be
                else{
                    continue;
                }
            }
        }

void CloudHandler::optimizationICP(){
    // pcl lib so slow :(  code myself
    // filterUsefulPoints();
    // // using pcl ICP
    // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // icp.setMaximumIterations (8);
    // icp.setInputSource (UsefulPoints1);
    // icp.setInputTarget (UsefulPoints2);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr  alignedPC;
    // alignedPC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // icp.align(*alignedPC);
    // Eigen::Matrix4f transform = icp.getFinalTransformation();
    // std::cout<<"rotation = "<<acos(transform(0,0))/3.14159*180<<std::endl;
    // pubPclCloud( alignedPC, &pubUsefulPoints1, & mapHeader );

    int totalIteration=0;
    if(initialized){
        totalIteration=icp_iteration;
    }
    else{
        totalIteration=icp_init_iteration;
    }
    // ICP iterations
    for (int iteration=0;iteration<totalIteration;iteration++){
        // if angle changes too much, needs to recalculate intersection with map, since there may be lots of mismatch by now.
        clock_t startTime,endTime;
        startTime = ros::Time::now().toNSec();
        averDistancePairedPoints=0;
        currentIteration=iteration;
        Vec_pcx.clear();
        Vec_pcy.clear();
        Vec_pedalx.clear();
        Vec_pedaly.clear();

        filterUsefulPoints();
        endTime = ros::Time::now().toNSec();
        cout << "The filterUsefulPoints run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
        startTime = ros::Time::now().toNSec();

        if(detect_corridor){
            auto startTime = ros::Time::now().toNSec();
            ROS_ERROR(" DETECT CORRIDOR???????????????");
            mergeMapHistogram();
            auto endTime = ros::Time::now().toNSec();
            cout << "The mergeMapHistogram run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
        }
        if(use_weight){
            mapCenter=mapCenter/weightSumTurkey;
            PCCenter=PCCenter/weightSumTurkey;          
            // std::cout<<"weighted map center = "<<mapCenter<<", weighted PCCenter = "<<PCCenter<<"weight = "<<weightSumTurkey<<std::endl;
        }else{
            mapCenter=mapCenter/numIcpPoints;
            PCCenter=PCCenter/numIcpPoints;
        }
        Eigen::Matrix2d W;
        W.setZero();
        auto startTime_ = ros::Time::now().toNSec();

        for (int i=0;i<numIcpPoints;i++){
            Eigen::Vector2d PCVec;
            Eigen::Vector2d MapVec;
            if(UsefulPoints1->points[usefulIndex[i]].x!=0||UsefulPoints1->points[usefulIndex[i]].y!=0){
                if(use_weight&&initialized){
                    PCVec<<UsefulPoints1->points[usefulIndex[i]].x,UsefulPoints1->points[usefulIndex[i]].y;
                    // PCVec<<UsefulPoints1->points[usefulIndex[i]].x-PCCenter(0),UsefulPoints1->points[usefulIndex[i]].y-PCCenter(1);
                    MapVec<<UsefulPoints2->points[usefulIndex[i]].x,UsefulPoints2->points[usefulIndex[i]].y;
                    W+=weightsTurkey[i]*MapVec*PCVec.transpose();
                }else{
                    PCVec<<UsefulPoints1->points[usefulIndex[i]].x-PCCenter(0),UsefulPoints1->points[usefulIndex[i]].y-PCCenter(1);
                    MapVec<<UsefulPoints2->points[usefulIndex[i]].x-mapCenter(0),UsefulPoints2->points[usefulIndex[i]].y-mapCenter(1);
                    W+=MapVec*PCVec.transpose();
                }
            }
        }

        if(use_weight&&initialized){
            W=1/weightSumTurkey*W-mapCenter*PCCenter.transpose();
            // W=1/weightSumTurkey*W;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU |Eigen::ComputeFullV);
        Eigen::Matrix2d U=svd.matrixU();
        Eigen::Matrix2d V=svd.matrixV();
        Eigen::Matrix2d rotationMatrix=U*V.transpose();
        Eigen::Vector2d translation=mapCenter-rotationMatrix*PCCenter;
        auto endTime_ = ros::Time::now().toNSec();
        cout << "The for loop +svd run time is:" << (double)(endTime_ - startTime_) / 1e6 << "ms" << endl;
        // if(initialized&&translation.norm()<translation_thres&&errorLowThredCurr>0.15&&errorUpThredCurr>0.15&&averDistancePairedPoints<1.5*errorUpThred){
        //     errorLowThredCurr=errorLowThredCurr/1.5;
        //     errorUpThredCurr=errorUpThredCurr/1.5;
        // }
        // accumulateAngle+=acos(rotationMatrix(0,0))/3.14159*180;
        // std::cout<<"----------------------------------------iteration = "<<iteration<<std::endl;
        startTime_ = ros::Time::now().toNSec();
        ROS_WARN("---------------iteration =, %d",iteration);
        // std::cout<<"translation = "<<std::endl<<translation<<std::endl;
        std::cout<<"translation norm = "<<translation.norm()<<"threshold = "<<errorLowThredCurr<<std::endl;
        pubPclCloud( UsefulPoints1, &pubUsefulPoints1, & mapHeader );
        pubPclCloud( UsefulPoints2, &pubUsefulPoints2, & mapHeader );

        // Eigen::Matrix4f robotPose;
        // auto   inverse_start = ros::Time::now().toNSec();
        Eigen::Matrix4f robotPoseOldInv=robotPose.inverse();
        // auto inverse_end = ros::Time::now().toNSec();
            // cout << "The inverse run time is:" << (double)(inverse_end - inverse_start) / 1e6 << "ms" << endl;
        robotPose(0,3)+=translation(0);
        robotPose(1,3)+=translation(1);
        robotPose(3,3)=1;
        robotPose.topLeftCorner(2,2)=rotationMatrix.cast<float>()*robotPose.topLeftCorner(2,2);
        robotPose(2,2)=1;
        endTime_ = ros::Time::now().toNSec();
        cout << "rest for loop +svd run time 0 is:" << (double)(endTime_ - startTime_) / 1e6 << "ms" << endl;
        //transform to lidar frame
        pcl::transformPointCloud(*transformed_pc,*transformed_pc,robotPose*robotPoseOldInv);

        endTime_ = ros::Time::now().toNSec();
        cout << "rest for loop +svd run time 1 is:" << (double)(endTime_ - startTime_) / 1e6 << "ms" << endl;
        //reset useful  points for next ICP iteration
        // UsefulPoints1->clear();
        // UsefulPoints2->clear();
        // potentialCeilingPoints->clear();
        //case3 and case2 after rescue
        // if(initialized||(!bTestRescue&&!bRescueRobot)){
            if(iteration%3==0){
        UsefulPoints1->points.resize(N_SCAN*Horizon_SCAN,0);
        UsefulPoints2->points.resize(N_SCAN*Horizon_SCAN,0);
            }

        // potentialCeilingPoints->points.resize(N_SCAN*Horizon_SCAN);
        endTime_ = ros::Time::now().toNSec();
        cout << "rest for loop +svd run time 2 is:" << (double)(endTime_ - startTime_) / 1e6 << "ms" << endl;
        // }
        // else{
        //     UsefulPoints1->points.resize(Horizon_SCAN);
        //     UsefulPoints2->points.resize(Horizon_SCAN);
        //     potentialCeilingPoints->points.resize(Horizon_SCAN);
        // }
        if(pause_iter){
            while(1){
                std::cout<<"press enter to continue "<<std::endl;
                std::getchar();
                break;
            }
        } 
        //if angle and translation donot change much, break
        // if(translation.norm()<0.01&&acos(rotationMatrix(0,0))/3.14159*180<0.02&&averDistancePairedPoints<0.1){
        // if((!bTestRescue)&& translation.norm()<icp_stop_translation_thred&&acos(rotationMatrix(0,0))/3.14159*180<icp_stop_rotation_thred){
            //FIXME: why nan?????????
            endTime_ = ros::Time::now().toNSec();
            cout << "rest for loop +svd run time is:" << (double)(endTime_ - startTime_) / 1e6 << "ms" << endl;
            endTime = ros::Time::now().toNSec();
            cout << "The EXCEPT filterUsefulPoints run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
            if(std::isnan(translation.norm())||translation.norm()<icp_stop_translation_thred&&acos(rotationMatrix(0,0))/3.14159*180<icp_stop_rotation_thred){
                if(!bTestRescue){
                    initialized=true;
                }
                break;
            }
        // if(pause_iter){
        //    while(1){
        //     std::cout<<"press enter to continue "<<std::endl;
        //     std::getchar();
        //     break;
        //    }
        // }
        endTime = ros::Time::now().toNSec();
        cout << "The EXCEPT filterUsefulPoints run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
    }// end of icp iterations
        // publish robot pose after icp iteration
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = mapHeader;
        pose_stamped.pose.position.x = robotPose(0,3);
        pose_stamped.pose.position.y = robotPose(1,3);
        pose_stamped.pose.position.z = 0;
        Eigen::Matrix3d rotation3d;
        rotation3d.setIdentity();
        rotation3d.topLeftCorner(3,3)=robotPose.topLeftCorner(3,3).cast<double>();
        Eigen::Quaterniond quaternion(rotation3d);
        pose_stamped.pose.orientation.x = quaternion.x();
        pose_stamped.pose.orientation.y = quaternion.y();
        pose_stamped.pose.orientation.z = quaternion.z();
        pose_stamped.pose.orientation.w = quaternion.w();
        globalPath.poses.push_back(pose_stamped);
        pubRobotPath.publish(globalPath);
        saveTUMTraj(pose_stamped);

    // std::fill(mapHistogram.begin(), mapHistogram.end(), 0);
    // only publish transformed_pc after whole icp
}

// TODO: use c++ map to see if it is faster
// after documenting each lidar point intersect with which map line, this function is used to merge these map lines according to similar angle for corridor detection 
void CloudHandler::mergeMapHistogram(){
    std::vector<int> usefulIndexHistogram;
    std::vector<double> weightsTurkeyHistogram;

    double intervalDeg=5;
    int interval=ceil(180/intervalDeg);
    std::vector<double> histogram(interval,0);
    //每一行代表这个角度，这一行的列代表组成这个角度的line的index
    //记录下每个mapline的角度，把这条mapline上hit的points数加到histogram上
    //mapHistogram, 与某一条mapline相交的point有多少个
    //histogram 与某个角度范围内的mapline相交的点有多少个
    //std::cout<<"mapHistogram  = ";
    double corridor_ness=0;
    int total_hit_points=0;

    auto  startTime = ros::Time::now().toNSec();
    for (int i=0;i<map_pc->points.size();i++){
        double angle = atan((map_pc->points[i].y-map_pc->points[(i+1)%map_pc->points.size()].y)/(map_pc->points[i].x-map_pc->points[(i+1)%map_pc->points.size()].x));
        //from [-90,90] to [0,180]
        angle = (angle+M_PI/2)/M_PI*180;
        int    index = floor(angle/intervalDeg);
        histogram[index]+=mapHistogram[i];
        // std::cout<<mapHistogram[i]<<",";
        total_hit_points+=mapHistogram[i];
        // corridor_ness+=mapHistogram[i]*angle;
    }
    auto endTime = ros::Time::now().toNSec();
    // cout << "The mergeMapHistogram for1 run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
    //perfect environment =0
    // corridor_ness=abs(corridor_ness-45*total_hit_points)/total_hit_poin;
    // ROS_WARN("================CORRIDOR NESS = %f",corridor_ness);
    int max_value=0;
    int max_index;
    int total_points=0;
    // std::cout<<"merged histogram = ";
    //找到最多点的角度
    startTime = ros::Time::now().toNSec();

    for(int i=0;i<histogram.size();i++){
        total_points+=histogram[i];
        if(histogram[i]>max_value){
            max_value=histogram[i];
            max_index=i;
        }
        // std::cout<<histogram[i]<<",";
    }
    endTime = ros::Time::now().toNSec();
    // cout << "The mergeMapHistogram for2 run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
    // 找到最多点对应的角度的全部mapline
    std::vector<int > mapLineIndex;
    startTime = ros::Time::now().toNSec();

    for (int i=0;i<map_pc->points.size();i++){
        double angle = atan((map_pc->points[i].y-map_pc->points[(i+1)%map_pc->points.size()].y)/(map_pc->points[i].x-map_pc->points[(i+1)%map_pc->points.size()].x));
        angle=(angle+M_PI/2)/M_PI*180;
        int index=floor(angle/intervalDeg);
        if(index==max_index&&mapHistogram[i]!=0){
            mapLineIndex.push_back(i);
        }
    }
    endTime = ros::Time::now().toNSec();
        // cout << "The mergeMapHistogram for3 run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
    double maxPercentage=double(max_value)/double(total_points+0.001);
    std::cout<<"total_points= "<<total_points<<" maxPercentage = "<<maxPercentage<<std::endl;
    double DSrate=corridornessDSRate(maxPercentage);
    ROS_WARN("CORRIDORNESS DOWNSAMPLE RATE = %f, maxPercentage= %f",DSrate,maxPercentage);
    if(DSrate>maxPercentageCorridor){
        // ROS_WARN("INSIDE CORRIDOR");
        onlyOneDirection=true;
        // numIcpPoints=0;
        // std::cout<<"!!!!!!!!!!!!!!!!!!!!! x= "<<dominantCloud->points[0].x<<std::endl;
        std::cout<<"weightSumTurkey before = "<<weightSumTurkey<<std::endl;
        startTime = ros::Time::now().toNSec();
        
        int temp_times=0;
        int minus_times=0;
        for(int i=0;i<usefulIndex.size();i++){
            bool find=false;
                for(int j=0;j<mapLineIndex.size();j++){
                    if(mapLineIndex[j]==int(UsefulPoints1->points[usefulIndex[i]].intensity)){
                        find=true;
                        // double distance=sqrt((Vec_pcx[i]-robotPose(0,3))*(Vec_pcx[i]-robotPose(0,3))+(Vec_pcy[i]-robotPose(1,3))*(Vec_pcy[i]-robotPose(1,3)));
                        double distance=organizedCloudIn->points[usefulIndex[i]].intensity;
                        if(distance<corridorDSmaxDist){
                            //dominant中要留下来的点
                            // if(i%subSample==0){
                            if((int)(i/DSrate)!=temp_times){
                                temp_times=(int)(i/DSrate);
                                // dominantCloud->points[i]=UsefulPoints1->points[usefulIndex[i]];
                                usefulIndexHistogram.push_back(usefulIndex[i]);
                                // remainedCloud->points[i]=UsefulPoints1->points[usefulIndex[i]];
                                weightsTurkeyHistogram.push_back(weightsTurkey[i]);
                            }
                            //dominant 中要被去掉的点
                            else{
                                vbHistogramRemain[usefulIndex[i]]=false;
                                numIcpPoints--;
                                UsefulPoints1->points[usefulIndex[i]].x=0;
                                UsefulPoints1->points[usefulIndex[i]].y=0;
                                UsefulPoints1->points[usefulIndex[i]].z=0;
                                UsefulPoints1->points[usefulIndex[i]].intensity=-1;
                                // startTime = ros::Time::now().toNSec();
                                if(use_weight&&initialized){
                                    PCCenter(0)-=Vec_pcx[i];
                                    PCCenter(1)-=Vec_pcy[i];
                                    mapCenter(0)-=Vec_pedalx[i];
                                    mapCenter(1)-=Vec_pedaly[i];
                                    weightSumTurkey-=weightsTurkey[i];
                                    minus_times++;
                                }
                                else{
                                    PCCenter(0)-=Vec_pcx[i];
                                    PCCenter(1)-=Vec_pcy[i];
                                    mapCenter(0)-=Vec_pedalx[i];
                                    mapCenter(1)-=Vec_pedaly[i];
                                    minus_times++;
                                }
                            }
                        }
                        else{
                            // dominantCloud->points[i]=UsefulPoints1->points[usefulIndex[i]];
                            //after corridorness downsample, remain points index
                            usefulIndexHistogram.push_back(usefulIndex[i]);
                            // remainedCloud->points[i]=UsefulPoints1->points[usefulIndex[i]];
                            weightsTurkeyHistogram.push_back(weightsTurkey[i]);
                        }
                    }
                }
                if(!find){
                    // remainedCloud->points.push_back(UsefulPoints1->points[i]);
                    // remainedCloud->points[i]=UsefulPoints1->points[usefulIndex[i]];
                    usefulIndexHistogram.push_back(usefulIndex[i]);
                    weightsTurkeyHistogram.push_back(weightsTurkey[i]);
                }
        }
        endTime = ros::Time::now().toNSec();
        // cout << "The mergeMapHistogram for4 run time is:" << (double)(endTime - startTime) / 1e6 << "ms, minus times"<<minus_times << endl;
        std::cout<<"weightSumTurkey before = "<<weightSumTurkey<<std::endl;

        usefulIndex=usefulIndexHistogram;
        weightsTurkey=weightsTurkeyHistogram;
        // pcl::VoxelGrid<pcl::PointXYZI> voxel;
        // voxel.setInputCloud(dominantCloud);
        // voxel.setLeafSize(20.0f, 20.0f, 20.0f);
        // voxel.filter(*dominantCloud);
    }
    // std::fill(mapHistogram.begin(), mapHistogram.end(), 0);
}

void CloudHandler::allocateMemory(){
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    laserUppestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    potentialCeilingPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    potentialCeilingPoints->points.resize(N_SCAN*Horizon_SCAN);
    organizedCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
    organizedCloudIn64.reset(new pcl::PointCloud<pcl::PointXYZI>());

    transformed_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    UsefulPoints1.reset(new pcl::PointCloud<pcl::PointXYZI>());
    UsefulPoints2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());
    mapCorridorEnlarge_pc.reset(new pcl::PointCloud<pcl::PointXYZI>());

    ringMapP1.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ringMapP1->points.resize(Horizon_SCAN);
    ringMapP2.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ringMapP2->points.resize(Horizon_SCAN);
    intersectionOnMap.reset(new pcl::PointCloud<pcl::PointXYZI>());
    intersectionOnMap->points.resize(Horizon_SCAN);
    furthestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    furthestRing->points.resize(Horizon_SCAN);
    transformedFurthestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    transformedFurthestRing->points.resize(Horizon_SCAN);
    insidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    insidePC->points.resize(Horizon_SCAN);
    outsidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    outsidePC->points.resize(Horizon_SCAN);

    organizedCloudIn->points.resize(N_SCAN*Horizon_SCAN);
    organizedCloudIn64->points.resize(64*Horizon_SCAN);

    transformed_pc->points.resize(N_SCAN*Horizon_SCAN);
    UsefulPoints1->points.resize(N_SCAN*Horizon_SCAN);
    UsefulPoints2->points.resize(N_SCAN*Horizon_SCAN);
    std::cout<<"allocate organize size= "<<organizedCloudIn->points.size()<<"intersectionOnMap size = "<<intersectionOnMap->points.size()<<std::endl;;
}

//called every frame and every guess
void CloudHandler::resetParameters(){
    laserCloudIn->clear();
    UsefulPoints1->clear();
    UsefulPoints2->clear();
    // potentialCeilingPoints->clear();
    // potentialCeilingPoints->points.resize(N_SCAN*Horizon_SCAN);
    // laserUppestRing->clear();
    // potentialCeilingPoints->clear();
    // transformed_pc->clear();
    // map_pc->clear();
    // ringMapP1->clear();
    ringMapP1->points.resize(Horizon_SCAN,0);
    // ringMapP2->clear();
    ringMapP2->points.resize(Horizon_SCAN,0);

    intersectionOnMap->clear();
    intersectionOnMap->points.resize(Horizon_SCAN,0);
    numIcpPoints=0;
    furthestRing->clear();
    furthestRing->points.resize(Horizon_SCAN);
    // intersectionOnMap->clear();
    intersectionOnMap->points.resize(Horizon_SCAN,0);
    organizedCloudIn->clear();

    organizedCloudIn->points.resize(N_SCAN*Horizon_SCAN,0);
    organizedCloudIn64->clear();
    organizedCloudIn64->points.resize(N_SCAN*Horizon_SCAN,0);

    transformed_pc->clear();
    transformed_pc->points.resize(N_SCAN*Horizon_SCAN,0);
    UsefulPoints1->points.resize(N_SCAN*Horizon_SCAN,0);
    UsefulPoints2->points.resize(N_SCAN*Horizon_SCAN,0);
}

// //map and bag are not recorded in the same time, therefore bag's lio sam path has a transform with line map
// void CloudHandler::liosamPathCB (const nav_msgs::Path& pathMsg){

// // -0.080	0.997	-0.000	0.241
// // -0.008	-0.000	1.000	-0.010
// // 0.000	0.000	0.000	1.000
// // [ 0, 0.0040028, -0.0400277, 0.9991906 ]

    
    
// //     0.9965   	-0.0800   	-0.0080   	-0.1024   
// // 0.0800   	0.9966   	-0.0006   	-0.2499   
// // 0.0080   	-0.0006   	0.9999   	0.0092   
// // 0.0000   	0.0000   	0.0000   	1.0000   
//     geometry_msgs::PoseStamped this_pose_stamped;
//     this_pose_stamped=*(pathMsg.poses.end()-1);

//     Eigen::Matrix3d rotation_matrix;
//     rotation_matrix<<0.9965  ,	-0.0800   ,	-0.0080,
//                                             0.0800  , 	0.9966   ,	-0.0006,
//                                             0.0080  , 	-0.0006   	,0.9999;

//     Eigen::Quaterniond quaternion(rotation_matrix);
//     Eigen::Quaterniond quaternionLIO(this_pose_stamped.pose.orientation.w,this_pose_stamped.pose.orientation.x,this_pose_stamped.pose.orientation.y,this_pose_stamped.pose.orientation.z);
//     Eigen::Quaterniond afterRotationQuat=quaternion*quaternionLIO;
//     double newx= 0.9965*(pathMsg.poses.end()-1)->pose.position.x - 0.08*	(pathMsg.poses.end()-1)->pose.position.y-0.0080*(pathMsg.poses.end()-1)->pose.position.z - 0.1024  ;
//     double newy=0.0800*(pathMsg.poses.end()-1)->pose.position.x+0.9966*	(pathMsg.poses.end()-1)->pose.position.y-0.0006*(pathMsg.poses.end()-1)->pose.position.z - 0.2499;
//     double newz=0.0080*(pathMsg.poses.end()-1)->pose.position.x	-0.0006*	(pathMsg.poses.end()-1)->pose.position.y+0.9999*(pathMsg.poses.end()-1)->pose.position.z+0.0092  ;

//     this_pose_stamped.pose.position.x = newx;
//     this_pose_stamped.pose.position.y = newy;
//     this_pose_stamped.pose.position.z = newz;
//     this_pose_stamped.pose.orientation.w=afterRotationQuat.w();
//     this_pose_stamped.pose.orientation.x=afterRotationQuat.x();
//     this_pose_stamped.pose.orientation.y=afterRotationQuat.y();
//     this_pose_stamped.pose.orientation.z=afterRotationQuat.z();


//     // this_pose_stamped.pose.position.y = pathMsg.end()->y;

//     // geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
//     // this_pose_stamped.pose.orientation.x = goal_quat.x;
//     // this_pose_stamped.pose.orientation.y = goal_quat.y;
//     // this_pose_stamped.pose.orientation.z = goal_quat.z;
//     // this_pose_stamped.pose.orientation.w = goal_quat.w;

//     // this_pose_stamped.header.stamp=current_time;
//     // this_pose_stamped.header.frame_id="odom";
//     TransformedLiosamPath.poses.push_back(this_pose_stamped);

//     pubTransformedLiosamPath.publish(TransformedLiosamPath);

// }
int main(int argc, char** argv)
{
    ros::init(argc, argv, "CloudHandler");

    CloudHandler cloudHandler;
    // CloudInitializer cloudinit;

    //cloudHandler.test();
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    //ROS_INFO("\033[1;32m----> CloudHandler Started.\033[0m");
    //三个sub，三个thread
    //ros::MultiThreadedSpinner spinner(2);
    ros::spin();
    
    return 0;
}
