#include"cloudInitializer.hpp"
#include <ros/master.h>
// publish to rviz showing info, for debugging
void CloudInitializer:: showImgIni(double x,double y,int yaw){
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("Things2say", 1);
    cv::Mat image(200, 600, CV_8UC3, cv::Scalar(0,0,0));
    // cv::Mat image = cv::imread("/home/xiefujing/research/area_graph/fujingL/Figures/area_graph.png", CV_LOAD_IMAGE_COLOR);
    string text1="Initializing,";
    string text2="current best guess: ";
    string text3="x="+to_string(x).substr(0,to_string(y).size()-5)+", y="+to_string(y).substr(0,to_string(y).size()-5)+", yaw="+to_string(yaw).substr(0,to_string(y).size()-5);
    putText(image,text1,cv::Point(20,60),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(255,255,255),1,8);
    putText(image,text2,cv::Point(20,100),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(255,255,255),1,8);
    putText(image,text3,cv::Point(20,140),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(255,255,255),1,8);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(mapHeader, "bgr8", image).toImageMsg();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    pub.publish(msg);
    ROS_ERROR("THINGS TO SAY!!!!!!!!!!!!!!!!!!!!!! ini");
}

CloudInitializer::CloudInitializer(){
    cout<<"start of construct main CloudInitializer"<<endl;
    bGuessReady=false;
    MaxRobotPose.setZero();
    MaxScore=0;
    pubRobotGuessMarker = nh.advertise<geometry_msgs::PointStamped>("pubRobotGuessMarker", 1, true);
    pubRobotPoseAfterICP= nh.advertise<geometry_msgs::PointStamped>("pubRobotPoseAfterICP", 1, true);
    pubCurrentMaxRobotPose= nh.advertise<geometry_msgs::PointStamped>("pubCurrentMaxRobotPose", 1, true);

    // subInitialGuess    = nh.subscribe<sensor_msgs::PointCloud>("/particles_for_init",1, &CloudInitializer::getInitialExtGuess, this);
    // numofInsidePoints=0;
    // numofOutsidePoints=0;
    rescueRunTime=0;
    rescueTimes=0;
    allocateMemory();

}
void CloudInitializer::setLaserCloudin(pcl::PointCloud<pcl::PointXYZI>::Ptr furthestRing_,std_msgs::Header mapHeader_){
    *furthestRing=*furthestRing_;
    mapHeader=mapHeader_;
    mapSize=map_pc->points.size();
    numofInsidePoints=0;
    numofOutsidePoints=0;
}

void CloudInitializer::setMapPC(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pc_){
    *map_pc=*map_pc_;
}

//been called in get initial guess, once get a guess, calculate the score of the guess
void CloudInitializer::rescueRobot(){
    // showImg1line("Initializing");
    cout<<"Guess is ready, start rescue"<<endl;

    std::ostringstream ts;
    ts.precision(2);
    ts << std::fixed << mapHeader.stamp.toSec();
    string filename="/home/xiefujing/research/area_graph/ws/frameResult/"+ts.str()+"rescueRoom.txt";
    rescueRoomStream.open(filename, ofstream::out | ofstream::app);
    rescueRoomStream.setf(ios::fixed);
    rescueRoomStream.precision(2);
    while(!rescueRoomStream.good()){
       ROS_ERROR_ONCE("ERROR OPENING FILE");
    }
    double currentTime=cloudHeader.stamp.toSec();
    auto startC = std::chrono::high_resolution_clock::now();

    // while(!imu_buf.empty() && imu_buf.begin()->header.stamp.toSec()<currentTime - 0.01)
    //             imu_buf.pop_front();
    // sensor_msgs::Imu currentImu=imu_buf.front();

    // 如果用imu的yaw初始化
    if(initialization_imu){
        // ROS_ERROR("WTF IMU.........................................................");
        // Eigen::Matrix3d rotation_matrix;
        // Eigen::Quaterniond q_imu(currentImu.orientation.w, currentImu.orientation.x,currentImu.orientation.y, currentImu.orientation.z);
        // // 2022-11-14-19-07-34.bag 's imu data has a rotation with map, because the map was not built by the bag...
        // Eigen::Quaterniond q_init(0.262963,0.0144884,-0.0534178,-0.963217);
        // Eigen::Quaterniond q=q_init.inverse()*q_imu;
        // Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);
        // // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0)+diff_angle_init/180*M_PI,Eigen::Vector3d::UnitZ())); 
        // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ())); 

        // rotation_matrix=yawAngle;
        // std::cout<<" initialial yaw angle = "<<eulerAngle(0)/M_PI*180<<","<<eulerAngle(1)/M_PI*180<<","<<eulerAngle(2)/M_PI*180<<std::endl;
        // std::cout<<"q= "<<currentImu.orientation.w<<","<<currentImu.orientation.x<<","<<currentImu.orientation.y<<","<<currentImu.orientation.z<<std::endl;
        // // rotation_matrix=q.matrix();
        // // rotation_matrix=q.toRotationMatrix();
        // robotPose.topLeftCorner(3,3)=rotation_matrix.cast<float>();

        // initializedUsingMassCenter();
        // // after initializing both rotation and translation, transform pointcloud and publish
        // pcl::transformPointCloud(*organizedCloudIn,*transformed_pc,robotPose);
        // pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
        //  if(pause_iter){
        //         while(1){
        //             std::cout<<"press enter to continue "<<std::endl;
        //             std::getchar();
        //             break;
        //         }
        //     }
        // calClosestMapPoint();
        // // optimizationICP();

        // double diff_angle=acos(robotPose(0,0))/3.14159*180-initialYawAngle;
        // std::cout<<"initializing averDistancePairedPoints = "<<averDistancePairedPoints<<std::endl;
        // std::cout<<"numIcpPoints = "<<numIcpPoints<<std::endl;
        // std::cout<<"IcpPointsPercentage = "<<IcpPointsPercentage<<std::endl;

        // pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
        // // if(pause_iter){
        // //         while(1){
        // //             std::cout<<"press enter to continue "<<std::endl;
        // //             std::getchar();
        // //             break;
        // //         }
        // //     }
        // resetParameters();

    }
    else{ // if(bTestRescue||bRescueRobot)
        int try_time=360/rescue_angle_interval;
        pcl::PointCloud<pcl::PointXYZI>::Ptr  organizedCloudInDS;
        organizedCloudInDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
        // try_time=20;
        pcl::PointCloud<pcl::PointXYZI>::Ptr  transformed_pcDS;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
        downSizeFurthestRing.setLeafSize(scoreDownsampleRate, scoreDownsampleRate, scoreDownsampleRate); // for global map visualization
        downSizeFurthestRing.setInputCloud(furthestRing);
        downSizeFurthestRing.filter(*organizedCloudInDS);
        cout<<"downsample size = "<<organizedCloudInDS->points.size()<<endl;
        // #pragma omp parallel for num_threads(8)

        for(int i=0;i<try_time;i++){
            // *organizedCloudIn=*organizedCloudInRecord;
            for(int j=0;j<corridorGuess.size();j++){

                clock_t startTime,endTime;
                startTime = ros::Time::now().toNSec();
                geometry_msgs::PointStamped this_guess_stamped;
                // this_guess_stamped.header=mapHeader;
                this_guess_stamped.header.frame_id = "map";
                this_guess_stamped.point.x = corridorGuess[j][0];
                this_guess_stamped.point.y =corridorGuess[j][1];
                this_guess_stamped.point.z = 0;
                pubRobotGuessMarker.publish(this_guess_stamped);

                accumulateAngle=0;
                averDistancePairedPoints=0;
                numofInsidePoints=0;
                numofOutsidePoints=0;
                insideScore=0;
                outsideScore=0;
                insideTotalRange=0;
                outsideTotalScore=0;
                turkeyScore=0;

                Eigen::Vector3f tempinitialExtTrans;
                tempinitialExtTrans<< corridorGuess[j][0],corridorGuess[j][1],0;
                setInitialPose(int(initialYawAngle+i*rescue_angle_interval)%360,tempinitialExtTrans);
                pcl::transformPointCloud(*organizedCloudInDS,*transformed_pc,robotPose);
                // pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );

                // cout<<"inside_index: "<<corridorGuess[j][2]<<endl;
                pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
                if(bInitializationWithICP){
                    clock_t startTime,endTime;
                    if(pause_iter){
                        while(1){
                            std::cout<<"press enter to continue,before icp "<<std::endl;
                            std::getchar();
                            break;
                        }
                    }
                    startTime = ros::Time::now().toNSec();
                    initializationICP(corridorGuess[j][2]);
                    endTime = ros::Time::now().toNSec();
                    // cout << "The initializationICP run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
                    pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );

                }// end of binitializationWithICP
                else{
                    pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
                    clock_t startTime,endTime;
                    startTime = ros::Time::now().toNSec();

                // z aka [2]means which area this guess belongs to
                    calClosestMapPoint(corridorGuess[j][2]);

                    endTime = ros::Time::now().toNSec();
                    // cout << "The calClosestMapPoint run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
                }
                double result_angle =0;
                if(bInitializationWithICP){
                    Eigen::Matrix4d temp=robotPose.cast<double>();
                    Eigen::Matrix3d temp_=temp.topLeftCorner(3,3);
                    Eigen::Quaterniond quaternion(temp_);
                    Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(1,2,0);
                    // Eigen::Matrix3f temp=robotPose.topLeftCorner(3,3);
                    // Eigen::Vector3f eulerAngle=temp.eulerAngles(2,1,0);
                    result_angle=eulerAngle[1]/M_PI*180;
                }
                else{
                    result_angle=int(initialYawAngle+i*rescue_angle_interval)%360;
                }

                initialized=false;
                // if(!insideOldArea(corridorGuess[j][2])){
                //     resetParameters();
                //     continue;
                // }
                
                if(bGenerateResultFile){
                    rescueRoomStream<< mapHeader.stamp.toSec()<<","<<result_angle<<","
                                                            <<corridorGuess[j](0)<<","<<corridorGuess[j](1)<<","<<robotPose(0,3)<<","<<robotPose(1,3)<<","
                                                            <<numofInsidePoints<<","<<insideScore<<","<<numofOutsidePoints<<","<<outsideScore<<","<<insideTotalRange<<","<<outsideTotalScore<<","<<turkeyScore<<endl;
                }
                cout << fixed;
                // cout<<setprecision(2)<<mapHeader.stamp.toSec()<<","<<result_angle<<","
                //                                             <<corridorGuess[j](0)<<","<<corridorGuess[j](1)<<","<<robotPose(0,3)<<","<<robotPose(1,3)<<","
                //                                             <<numofInsidePoints<<","<<insideScore<<","<<numofOutsidePoints<<","<<outsideScore<<","<<insideTotalRange<<"___"<<insideTotalRange/(insideScore+outsideScore)<<","<<outsideTotalScore<<","<<turkeyScore<<endl;
                if(MaxScore<1.0/(insideScore+outsideScore)){
                    MaxScore=1.0/(insideScore+outsideScore);
                    MaxRobotPose=robotPose;
                    geometry_msgs::PointStamped pose_max_stamped;
                    pose_max_stamped.header.frame_id = "map";
                    pose_max_stamped.point.x = robotPose(0,3);
                    pose_max_stamped.point.y =robotPose(1,3);
                    pose_max_stamped.point.z = 0;
                    pubCurrentMaxRobotPose.publish(pose_max_stamped);
                    showImgIni(robotPose(0,3),robotPose(1,3),int(initialYawAngle+i*rescue_angle_interval)%360);

                }
                if(pause_iter){
                    while(1){
                        std::cout<<"press enter to continue , after icp"<<std::endl;
                        std::getchar();
                        break;
                    }
                }
                if(turkeyPauseThred>0&&turkeyScore>turkeyPauseThred){
                    while(1){
                        std::cout<<"press enter to continue "<<std::endl;
                        std::getchar();
                        break;
                    }
                }
                resetParameters();
                // if(insideTotalRange/(insideScore+outsideScore)>6.32){
                //     while(1){
                //         std::cout<<"press enter to continue "<<std::endl;
                //         std::getchar();
                //         break;
                //     }
                // }
                // cout<<" max robot pose = "<<MaxRobotPose(0,3)<<","<<MaxRobotPose(1,3)<<endl;
                endTime = ros::Time::now().toNSec();
                // cout << "one guess run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;

            }
            double diff_angle=acos(robotPose(0,0))/3.14159*180-initialYawAngle;
            // std::cout<<"-----------------------------------initializating, initial angle = "<<initialYawAngle+i*rescue_angle_interval<<std::endl;
            // pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
            // if(pause_iter){
            //     while(1){
            //         std::cout<<"press enter to continue "<<std::endl;
            //         std::getchar();
            //         break;
            //     }
            // }
        }//end of all guess for

        bRescueRobot=false;
        errorUpThred=3;
        if(bRescueRobot){

            // subInitialGuess    = nh.subscribe<sensor_msgs::PointCloud>("/none",1,&CloudInitializer:: getInitialExtGuess, this,ros::TransportHints().tcpNoDelay());
            system("rosnode kill particlesGeneratorNode");

        }

        auto finishC = std::chrono::high_resolution_clock::now();
        rescueTimes++;
        rescueRunTime+=std::chrono::duration_cast<std::chrono::nanoseconds>(finishC-startC).count();
        cout<<"average rescue run time = "<<rescueRunTime/rescueTimes<<"ns, rescueTimes = "<<rescueTimes<<endl;
        rescueRoomStream.flush();
        rescueRoomStream.close();
        geometry_msgs::Pose pose;
        pubDONEsignal.publish(pose);

        bGuessReady=false;

        // FIXME: get a single result of all assumptime.....
        // if(!bTestRescue){
        //     initialized=true;
        // }
    }

}
bool CloudInitializer::insideOldArea(int mapPCindex){
    int throughTimes=0;
    for(int i=mapPCindex; i<map_pc->points.size();i++){
        //end of a this area
        if((int)map_pc->points[i].intensity%3==2){
            break;
        }
        pcl::PointXYZI temp;
        temp.x=robotPose(0,3);
        temp.y=robotPose(1,3);
        pcl::PointXYZI temp_;
        temp_.x=robotPose(0,3)+1000;
        temp_.y=robotPose(1,3)+1000;
        bool inray;
        inRayGeneral(map_pc->points[i],map_pc->points[(i+1)%map_pc->points.size()],temp,temp_,inray);
        if(inray){
            throughTimes++;
        }
    }
    if(throughTimes%2==0){
        return false;
    }
    else{
        return true;
    }
}

void CloudInitializer::initializationICP(int insideAGIndex){
    int totalIteration=0;
    totalIteration=icp_init_iteration;
    Eigen::Matrix4f robotPoseGuess=robotPose;

    for (int iteration=0;iteration<totalIteration;iteration++){
        //reset center for each iteration
        PCCenter.setZero();
        mapCenter.setZero();

        averDistancePairedPoints=0;
        numofInsidePoints=0;
        numofOutsidePoints=0;
        insideScore=0;
        outsideScore=0;
        insideTotalRange=0;
        outsideTotalScore=0;
        turkeyScore=0;

        Vec_pcx.clear();
        Vec_pcy.clear();
        Vec_pedalx.clear();
        Vec_pedaly.clear();
        ROS_WARN("Useful POINTS SIZE 1 = ,%d",UsefulPoints1->points.size());

        calClosestMapPoint(insideAGIndex);
        ROS_WARN("Useful POINTS SIZE 2 = ,%d",UsefulPoints1->points.size());

        // if(detect_corridor){
        //     auto startTime = ros::Time::now().toNSec();
        //     ROS_ERROR(" DETECT CORRIDOR???????????????");
        //     mergeMapHistogram();
        //     auto endTime = ros::Time::now().toNSec();
        //     cout << "The mergeMapHistogram run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
        // }
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

        // std::cout<<"rotation = "<<acos(rotationMatrix(0,0))/3.14159*180<<"num of icp match points = "<<numIcpPoints<<"W/point= "<<W.norm()/numIcpPoints<<std::endl;
        // std::cout<<"rotation matrix = "<<std::endl<<rotationMatrix<<std::endl;
        // IcpPointsPercentage=double(numIcpPoints)/double(transformed_pc->points.size());
        // averDistancePairedPoints=averDistancePairedPoints/numIcpPoints;
        // std::cout<<"percentage = "<<IcpPointsPercentage<<", averDistancePairedPoints= "<<averDistancePairedPoints<<std::endl;
        // double yaw_angle=acos(rotationMatrix(0,0));
        // Eigen::AngleAxisd rotation_vector;
        // rotation_vector.fromRotationMatrix(rotationMatrix);
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
        UsefulPoints1->clear();
        UsefulPoints2->clear();
        UsefulPoints1->points.resize(Horizon_SCAN,0);
        UsefulPoints2->points.resize(Horizon_SCAN,0);
        // potentialCeilingPoints->clear();
        //case3 and case2 after rescue
        // if(initialized||(!bTestRescue&&!bRescueRobot)){
            // if(iteration%3==0){
            //     UsefulPoints1->points.resize(Horizon_SCAN,0);
            //     UsefulPoints2->points.resize(Horizon_SCAN,0);
            // }

        geometry_msgs::PointStamped pose_after_icp_stamped;
        pose_after_icp_stamped.header.frame_id = "map";
        pose_after_icp_stamped.point.x = robotPose(0,3);
        pose_after_icp_stamped.point.y =robotPose(1,3);
        pose_after_icp_stamped.point.z = 0;
        pubRobotPoseAfterICP.publish(pose_after_icp_stamped);

        endTime_ = ros::Time::now().toNSec();
        cout << "rest for loop +svd run time 2 is:" << (double)(endTime_ - startTime_) / 1e6 << "ms" << endl;
        if(checkICPmovingDist(robotPoseGuess)){
            return;
        }
        // if(pause_iter){
        //     while(1){
        //         std::cout<<"press enter to continue "<<std::endl;
        //         std::getchar();
        //         break;
        //     }
        // } 
        //if angle and translation donot change much, break
        // if(translation.norm()<0.01&&acos(rotationMatrix(0,0))/3.14159*180<0.02&&averDistancePairedPoints<0.1){
        // if((!bTestRescue)&& translation.norm()<icp_stop_translation_thred&&acos(rotationMatrix(0,0))/3.14159*180<icp_stop_rotation_thred){
            //FIXME: why nan?????????

        //     if(std::isnan(translation.norm())||translation.norm()<icp_stop_translation_thred&&acos(rotationMatrix(0,0))/3.14159*180<icp_stop_rotation_thred){
        //         if(!bTestRescue){
        //             initialized=true;
        //         }
        //         break;
        //     }
        // // if(pause_iter){
        //    while(1){
        //     std::cout<<"press enter to continue "<<std::endl;
        //     std::getchar();
        //     break;
        //    }
        // }

    }// end of iterations
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
        // if(initialized||iteration==0){
            globalPath.poses.push_back(pose_stamped);
        // }
        pubRobotPath.publish(globalPath);
        saveTUMTraj(  pose_stamped);

    // std::fill(mapHistogram.begin(), mapHistogram.end(), 0);
    // only publish transformed_pc after whole icp
}

bool CloudInitializer::checkICPmovingDist(Eigen::Matrix4f robotPoseGuess){
    return sqrt((robotPoseGuess(0,3)-robotPose(0,3))*(robotPoseGuess(0,3)-robotPose(0,3))+
    (robotPoseGuess(1,3)-robotPose(1,3))*(robotPoseGuess(1,3)-robotPose(1,3)))>5;
}

void CloudInitializer::calClosestMapPoint(int inside_index){
    // cout<<"CloudInitializer calClosestMapPoint"<<endl;
    int last_index=0;
    // use for rescue, how many times a ray goes through map
    size_t transformed_pc_size=transformed_pc->points.size();
    numIcpPoints=0;
    weightSumTurkey=0;
    weightSumCauchy=0;
    weightsTurkey.clear();
    outsideAreaIndexRecord.resize(transformed_pc->points.size(),0);
    outsideAreaLastRingIndexRecord.resize(Horizon_SCAN,0);

    numofIntersection.resize(transformed_pc_size);
    // cout<<"numofIntersection size = "<<numofIntersection.size()<<endl;
    std::fill(numofIntersection.begin(), numofIntersection.end(), 0);
    inRayDis.resize(transformed_pc_size);
    std::fill(inRayDis.begin(), inRayDis.end(), 0);
    inRayRange.resize(transformed_pc_size);
    std::fill(inRayRange.begin(), inRayRange.end(), 0);
    match_with_outside.resize(transformed_pc_size);
    std::fill(match_with_outside.begin(), match_with_outside.end(), 0);

    insidePC->clear();
    insidePC->points.resize(transformed_pc_size);
    outsidePC->clear();
    outsidePC->points.resize(transformed_pc_size);
    // #pragma omp parallel for num_threads(8)

    for(int i=0;i<transformed_pc_size;i++){
        bool findIntersection=false;
        //case3 and case2 after rescue
        //FIXME:
        // if(bOnlyScoreParticles){
            // checkMapinRay(0,i,last_index);
            double minDist=0;
            bool bDistanceInside;
            findIntersection=checkMap(0,i,last_index, minDist,inside_index);
            double pedalx;
            double pedaly;
            calPedal(ringMapP1->points[i].x,ringMapP1->points[i].y,ringMapP2->points[i].x,ringMapP2->points[i].y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
            double error=sqrt((pedalx-transformed_pc->points[i].x)*(pedalx-transformed_pc->points[i].x)+(pedaly-transformed_pc->points[i].y)*(pedaly-transformed_pc->points[i].y));
            // checkMapinRay(0,i,last_index);
            if(!findIntersection){
                ROS_ERROR_ONCE("NO MATCHING MAP INTERSECTION FOR THIS POINTS");
                continue;
            }
            if(inRayDis[i]<1e-6&&(transformed_pc->points[i].x!=0||transformed_pc->points[i].y!=0)&&findIntersection){
                inRayDis[i]=error;
                inRayRange[i]=sqrt(minDist);
            }
            else{
                cout<<"wtf"<<endl;
            }
        if(!findIntersection){
            intersectionOnMap->points[i].x=0;
            intersectionOnMap->points[i].y=0;
            intersectionOnMap->points[i].z=0;
        }

    }
    pubPclCloud( intersectionOnMap, &pubIntersection, & mapHeader );
    //PC points inside or outside of map
            // #pragma omp parallel for num_threads(8)

    for(int i=0;i<transformed_pc_size;i++){
        outsideTotalScore+=match_with_outside[i];
        // even number intersection with map, plus robot pose, ray has odd number of intersection with map, means inside map.
        // see:https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
        //DOING: handle contradictory, if the points are label as inside by the algorithm above, 
                            // but its pc length is longer than pose to map, we now it is using points in corridor to match room points.
        double pcx=transformed_pc->points[i].x;
        double pcy=transformed_pc->points[i].y;
        double pedalx;
        double pedaly;
        calPedal(ringMapP1->points[i].x,ringMapP1->points[i].y,ringMapP2->points[i].x,ringMapP2->points[i].y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
        
        if(transformed_pc->points[i].x!=0||transformed_pc->points[i].y!=0){        
            if(numofIntersection[i]%2==0&&intersectionOnMap->points[i].intensity==1){
                // cout<<numofIntersection[i]<<",";
                insidePC->points[i]=transformed_pc->points[i];
                numofInsidePoints++;
                double weight=calWeightTurkey(inRayDis[i],errorLowThred,false,errorUpThred);

                if(inRayDis[i]<50){
                    if(inRayDis[i]<0.8){
                        insideScore+=inRayDis[i];
                    }
                    else{
                        insideScore+=2;
                    }
                    turkeyScore+=weight;
                }
                insideTotalRange+=inRayRange[i];
                if(inRayDis[i]<errorLowThredInit){
                    numIcpPoints++;
                    usefulIndex.push_back(i);  
                    UsefulPoints1->points[i]=transformed_pc->points[i];
                    UsefulPoints2->points[i].x=pedalx;
                    UsefulPoints2->points[i].y=pedaly;
                    UsefulPoints2->points[i].z=transformed_pc->points[i].z;
                    weightSumTurkey+=weight;
                    weightsTurkey.push_back(weight);
                    if(bInitializationWithICP){
                        if(use_weight){
                            double tempx=weight*pcx;
                            double tempy=weight*pcy;
                            PCCenter(0)+=tempx;
                            PCCenter(1)+=tempy;
                            double tempx_=weight*pedalx;
                            double tempy_=weight*pedaly;
                            mapCenter(0)+=tempx_;
                            mapCenter(1)+=tempy_;

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
                    }

                }
            }
            else{
                outsidePC->points[i]=transformed_pc->points[i];
                numofOutsidePoints++;
                double weight=calWeightTurkey(inRayDis[i],errorLowThred,true,errorUpThred);
                if(inRayDis[i]<50){
                    turkeyScore+=weight;
                   if(inRayDis[i]<0.8){
                        outsideScore+=inRayDis[i];
                    }
                    else{
                        outsideScore+=2;
                    }
                    if(inRayDis[i]<errorUpThredInit){
                    numIcpPoints++;
                    usefulIndex.push_back(i);  
                    UsefulPoints1->points[i]=transformed_pc->points[i];
                    UsefulPoints2->points[i].x=pedalx;
                    UsefulPoints2->points[i].y=pedaly;
                    UsefulPoints2->points[i].z=transformed_pc->points[i].z;
                    weightSumTurkey+=weight;
                    weightsTurkey.push_back(weight);
                    if(bInitializationWithICP){
                        if(use_weight){
                            double tempx=weight*pcx;
                            double tempy=weight*pcy;
                            PCCenter(0)+=tempx;
                            PCCenter(1)+=tempy;
                            double tempx_=weight*pedalx;
                            double tempy_=weight*pedaly;
                            mapCenter(0)+=tempx_;
                            mapCenter(1)+=tempy_;

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
                    }
                }
                }
                if(inRayDis[i]<0.5){
                    insideTotalRange+=inRayRange[i];
                }
            }
        }
    }
        // cout<<"checking result: "<<numofInsidePoints<<","<<insideScore<<","<<numofOutsidePoints<<","<<outsideScore<<","<<insideTotalRange<<endl;
    if(bResultChecking){
        // cout<<"checking result: "<<numofInsidePoints<<","<<insideScore<<","<<numofOutsidePoints<<","<<outsideScore<<","<<insideTotalRange<<endl;
    }
    pubPclCloud( insidePC, &pubInsidePC, & mapHeader );
    pubPclCloud( outsidePC, &pubOutsidePC, & mapHeader );
}
//inside_index which area the robotpose is inside in
void CloudInitializer::checkWholeMap(const pcl::PointXYZI& PCPoint,const pcl::PointXYZI &PosePoint,int horizonIndex,double & minDist,bool& findIntersection){
    double min_error=0;
    double min_PCLength=0;
    double min_mapLength=0;
    for(int i=0;i<map_pc->size();i++){
        //TODO:maybe not necessary, for each guess, same number of inside to be ignore?????
        // bool bOnRay=false;
        // inRay(PosePoint,PCPoint,map_pc->points[i%mapSize],map_pc->points[(i+1)%mapSize],bOnRay);
        // if(bOnRay&&(PCPoint.x!=0||PCPoint.y!=0)){
        //     numofIntersection[horizonIndex]+=1;
        // }
        //since the end of a area should not be connected with the start of the next area.
        if(((int)map_pc->points[i%mapSize].intensity)%3==2){
            continue;
        }
        pcl::PointXYZI intersectionOnMapThisLine;
        bool inbetween=inBetween(PosePoint,PCPoint,map_pc->points[i%mapSize],map_pc->points[(i+1)%mapSize],&intersectionOnMapThisLine);

        if(inbetween){
            //chose the closest intersection                                
            double dist=calDistance(intersectionOnMapThisLine,PCPoint);
            if(min_error==0 ||min_error>dist){
                findIntersection=true;
                min_error=dist;
                // cout<<"size  = "<<intersectionOnMap->points.size()<<","<<ringMapP1->points.size()<<","<<ringMapP2->points.size()<<endl;
                intersectionOnMap->points[horizonIndex]=intersectionOnMapThisLine;
                //mark if pc length is shorter than map length
                double mapLength=calDistance(intersectionOnMapThisLine,PosePoint);
                double PCLength=calDistance(PCPoint,PosePoint);
                min_mapLength=mapLength;
                min_PCLength=PCLength;
                if(PCLength<mapLength){
                    // bDistanceInside=true;
                    intersectionOnMap->points[horizonIndex].intensity=1;
                    match_with_outside[horizonIndex]=0;

                }else{
                    // double pedalx;
                    // double pedaly;
                    // calPedal(map_pc->points[horizonIndex%mapSize].x,map_pc->points[j%mapSize].y,map_pc->points[(j+1)%mapSize].x,map_pc->points[(j+1)%mapSize].y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
                    // double error=sqrt((pedalx-transformed_pc->points[i].x)*(pedalx-transformed_pc->points[i].x)+(pedaly-transformed_pc->points[i].y)*(pedaly-transformed_pc->points[i].y));
                    // match_with_outside[horizonIndex]=error;
                    match_with_outside[horizonIndex]=PCLength-mapLength;

                }
                ringMapP1->points[horizonIndex]=map_pc->points[i%mapSize];
                ringMapP2->points[horizonIndex]=map_pc->points[(i+1)%mapSize];
                //BUG: 
                // last_index=j%mapSize;
                transformed_pc->points[horizonIndex].intensity=i%mapSize;
            }
        }
        else{
            continue;
        }
    }
    if(min_mapLength>min_PCLength){
        minDist=min_mapLength*min_mapLength;
    }
    else{
        minDist=0;
    }
}
bool CloudInitializer::checkMap(int ring,int horizonIndex,int& last_index,double & minDist,int inside_index){
    // cout<<"in check map"<<endl;
    pcl::PointXYZI PCPoint;
    PCPoint.x =transformed_pc->points[horizonIndex].x;
    PCPoint.y =transformed_pc->points[horizonIndex].y;
    PCPoint.z =0;
    pcl::PointXYZI PosePoint;
    PosePoint.x=robotPose(0,3);
    PosePoint.y=robotPose(1,3);
    PosePoint.z=0;

    bool findIntersection=false;
    minDist=0;
    //traverse map
    // auto start_index=AG_index.area_index[inside_index];
    // pcl::PointCloud<pcl::PointXYZI>::Ptr testarea(new pcl::PointCloud<pcl::PointXYZI>());
    // for each area, the end point is the same as the start point
    //4
    // testarea->resize(AG_index.area_index[inside_index].end-AG_index.area_index[inside_index].start);

    //no need to handle the last point, the last edge is handle
    //0    3
            #pragma omp parallel for num_threads(8)

    for(int j=AG_index.area_index[inside_index].start; j<AG_index.area_index[inside_index].end; j++){
        // for(int k=0;k<AG_index.area_index[inside_index].passage.size();k++){
        //     if(j==AG_index.area_index[inside_index].passage[k]){
        //         checkWholeMap(PCPoint,PosePoint, horizonIndex,  minDist, findIntersection);
        //         ROS_WARN("THIS IS A PASSAGE");
        //         break;
        //     }
        //     else{
        //         continue;
        //     }
        // }
        // if((int)map_pc->points[j%mapSize].intensity/3!=0&&horizonIndex==0){

        //         checkWholeMap(PCPoint,PosePoint, horizonIndex,  minDist, findIntersection);
        //         ROS_WARN("THIS IS A PASSAGE %f,%d",map_pc->points[j%mapSize].intensity,(int)map_pc->points[j%mapSize].intensity);
        //         continue;

        // }
        //     pcl::PointXYZI temp;
        // temp.x =transformed_pc->points[horizonIndex].x;
        // temp.y =transformed_pc->points[horizonIndex].y;
        // temp.z =0;
        // testarea->points[j-AG_index.area_index[inside_index].start]=map_pc->points[j%mapSize];
        //WRONG map point..........
        // if((j%mapSize==28)||(j%mapSize==32)){
        //     continue;
        // }
        //FIXME:
        // cout<<map_pc->points.size()<<","<<mapSize<<endl;
        bool bOnRay=false;
        inRay(PosePoint,PCPoint,map_pc->points[j%mapSize],map_pc->points[(j+1)%mapSize],bOnRay);
        if(bOnRay&&(PCPoint.x!=0||PCPoint.y!=0)){
            numofIntersection[horizonIndex]+=1;
        }
        pcl::PointXYZI intersectionOnMapThisLine;
        
        bool inbetween=inBetween(PosePoint,PCPoint,map_pc->points[j%mapSize],map_pc->points[(j+1)%mapSize],&intersectionOnMapThisLine);

        if(inbetween){
            //means this edge is passage, the robot could see through
            if((int)map_pc->points[j%mapSize].intensity>2&&(int)map_pc->points[(j+1)%mapSize].intensity>2&&(int)map_pc->points[j%mapSize].intensity%3!=2){
                // double pedalx;
                // double pedaly;
                // calPedal(map_pc->points[horizonIndex%mapSize].x,map_pc->points[j%mapSize].y,map_pc->points[(j+1)%mapSize].x,map_pc->points[(j+1)%mapSize].y,transformed_pc->points[i].x,transformed_pc->points[i].y,pedalx,pedaly);
                // double error=sqrt((pedalx-transformed_pc->points[i].x)*(pedalx-transformed_pc->points[i].x)+(pedaly-transformed_pc->points[i].y)*(pedaly-transformed_pc->points[i].y));
                // if(calDistance(PCPoint,PosePoint)>calDistance(intersectionOnMapThisLine,PosePoint)){
                //     match_with_outside[horizonIndex]=error;
                // }
                
                checkWholeMap(PCPoint,PosePoint, horizonIndex,  minDist, findIntersection);
                // ROS_WARN("THIS IS A PASSAGE %f,%d",map_pc->points[j%mapSize].intensity,(int)map_pc->points[j%mapSize].intensity);
                continue;
            }
            //chose the closest intersection                                
            if(minDist==0 ||
                        minDist>((intersectionOnMapThisLine.x-PosePoint.x)*(intersectionOnMapThisLine.x-PosePoint.x)+
                                            (intersectionOnMapThisLine.y-PosePoint.y)*(intersectionOnMapThisLine.y-PosePoint.y))){
                
                findIntersection=true;
                minDist=(intersectionOnMapThisLine.x-PosePoint.x)*(intersectionOnMapThisLine.x-PosePoint.x)+
                                    (intersectionOnMapThisLine.y-PosePoint.y)*(intersectionOnMapThisLine.y-PosePoint.y);
                // cout<<"size  = "<<intersectionOnMap->points.size()<<","<<ringMapP1->points.size()<<","<<ringMapP2->points.size()<<endl;
                intersectionOnMap->points[horizonIndex]=intersectionOnMapThisLine;
                //mark if pc length is shorter than map length
                double mapLength=calDistance(intersectionOnMapThisLine,PosePoint);
                double PCLength=calDistance(PCPoint,PosePoint);
                if(PCLength<mapLength){
                    // bDistanceInside=true;
                    intersectionOnMap->points[horizonIndex].intensity=1;
                }
                else{
                    match_with_outside[horizonIndex]=PCLength-mapLength;
                }
                ringMapP1->points[horizonIndex]=map_pc->points[j%mapSize];
                ringMapP2->points[horizonIndex]=map_pc->points[(j+1)%mapSize];
                //BUG: 
                last_index=j%mapSize;
                transformed_pc->points[horizonIndex].intensity=j%mapSize;
            }
        }
        else{
            continue;
        }
    }
    // pubPclCloud(testarea,&pubtest,&mapHeader);
    return findIntersection;
}
void CloudInitializer::scoreParticlesDist(){
    // double currentTime=cloudHeader.stamp.toSec();
    int try_time=360/rescue_angle_interval;
    pcl::PointCloud<pcl::PointXYZI>::Ptr  organizedCloudInDS;
    organizedCloudInDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // *organizedCloudInDS=*furthestRing;

    try_time=20;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr  transformed_pcDS;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
    downSizeFurthestRing.setLeafSize(scoreDownsampleRate, scoreDownsampleRate, scoreDownsampleRate); // for global map visualization
    downSizeFurthestRing.setInputCloud(furthestRing);
    downSizeFurthestRing.filter(*organizedCloudInDS);
    cout<<"downsample size = "<<organizedCloudInDS->points.size()<<endl;

    for(int i=0;i<try_time;i++){
        // *organizedCloudIn=*organizedCloudInDS;
        for(int j=0;j<corridorGuess.size();j++){
            insideScore=0;
            outsideScore=0;
            insideTotalRange=0;
            numofInsidePoints=0;
            numofOutsidePoints=0;
            
            setInitialPose(int(initialYawAngle+i*rescue_angle_interval)%360,corridorGuess[j]);
            pcl::transformPointCloud(*organizedCloudInDS,*transformed_pc,robotPose);
            pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
            cout<<"size of transformed pc"<<transformed_pc->points.size()<<endl;
            // if(pause_iter){
            //     while(1){
            //         std::cout<<"press enter to continue "<<std::endl;
            //         std::getchar();
            //         break;
            //     }
            // }
            // initializedUsingMassCenter();
            clock_t startTime,endTime;
            startTime = ros::Time::now().toNSec();
            calClosestMapPoint(j);
            endTime = ros::Time::now().toNSec();
            cout << "The calClosestMapPoint run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;

            // startTime = ros::Time::now().toNSec();
            // optimizationICP();
            // endTime = ros::Time::now().toNSec();
            // cout << "The optimizationICP run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;

            initialized=false;

            rescueRoomStream<< mapHeader.stamp.toSec()<<","<<int(initialYawAngle+i*rescue_angle_interval)%360<<","
                <<corridorGuess[j](0)<<","<<corridorGuess[j](1)<<","<<robotPose(0,3)<<","<<robotPose(1,3)<<","
                <<numofInsidePoints<<","<<insideScore<<","<<numofOutsidePoints<<","<<outsideScore<<","<<insideTotalRange<<endl;
            resetParameters();
            // if(pause_iter){
            //     while(1){
            //         std::cout<<"press enter to continue "<<std::endl;
            //         std::getchar();
            //         break;
            //     }
            // }
            endTime = ros::Time::now().toNSec();
            cout << "one guess run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
        }
        double diff_angle=acos(robotPose(0,0))/3.14159*180-initialYawAngle;
        std::cout<<"-----------------------------------scoring, initial angle = "<<initialYawAngle+i*rescue_angle_interval<<std::endl;

        pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
           
     }
    // FIXME: get a single result of all assumptime.....
    cout<<"end scoreParticles"<<endl;
    rescueRoomStream.close();
    geometry_msgs::Pose pose;
    pubDONEsignal.publish(pose);
}

CloudInitializer::~CloudInitializer(void){
    cout << "CloudInitializer is being deleted" << endl;
}

void CloudInitializer::getInitialExtGuess(const sensor_msgs::PointCloudConstPtr& laserCloudMsg ){
    ROS_WARN("CloudInitializer GETTING INITIALI GUESS");

    int GuessSize = laserCloudMsg->points.size(); 
    corridorGuess.clear();
    for (int i = 0; i < GuessSize; i++){
        Eigen::Vector3f tempGuess;
        //z means which area this guess belongs to
        tempGuess<< laserCloudMsg->points[i].x, laserCloudMsg->points[i].y,laserCloudMsg->points[i].z;
        corridorGuess.push_back(tempGuess);
    }
    bGuessReady=true;
    clock_t startTime,endTime,startTimecb;
    startTime = ros::Time::now().toNSec();

    rescueRobot();
    robotPose=MaxRobotPose;
    endTime = ros::Time::now().toNSec();
    cout <<"number of guess is = "<<GuessSize<< ", The rescueRobot_run_time is " << (double)(endTime - startTime) / 1e6 << "ms" << endl;
    // showImg1line("Pose tracking");
}

void CloudInitializer::scoreParticles(){
    double currentTime=cloudHeader.stamp.toSec();
    if(bTestRescue||bRescueRobot){
        int try_time=360/rescue_angle_interval;
        pcl::PointCloud<pcl::PointXYZI>::Ptr  organizedCloudInRecord;
        organizedCloudInRecord.reset(new pcl::PointCloud<pcl::PointXYZI>());
        // *organizedCloudInRecord=*furthestRing;
        // 
        // try_time=20;
        // HACK:
        // try_time=1;
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr  transformed_pcDS;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
        downSizeFurthestRing.setLeafSize(scoreDownsampleRate, scoreDownsampleRate, scoreDownsampleRate); // for global map visualization
        downSizeFurthestRing.setInputCloud(furthestRing);
        downSizeFurthestRing.filter(*organizedCloudInRecord);
        cout<<"downsample size = "<<organizedCloudInRecord->points.size()<<endl;

        for(int i=0;i<try_time;i++){
            // *organizedCloudIn=*organizedCloudInRecord;
            for(int j=0;j<corridorGuess.size();j++){
                insideScore=0;
                outsideScore=0;
                insideTotalRange=0;
                numofInsidePoints=0;
                numofOutsidePoints=0;
                setInitialPose(int(initialYawAngle+i*rescue_angle_interval)%360,corridorGuess[j]);
                pcl::transformPointCloud(*organizedCloudInRecord,*transformed_pc,robotPose);
                pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
                cout<<"size of transformed pc"<<transformed_pc->points.size()<<endl;
                // if(pause_iter){
                //     while(1){
                //         std::cout<<"press enter to continue "<<std::endl;
                //         std::getchar();
                //         break;
                //     }
                // }
                // initializedUsingMassCenter();
                clock_t startTime,endTime;
                startTime = ros::Time::now().toNSec();
                calClosestMapPoint(j);
                endTime = ros::Time::now().toNSec();
                cout << "The calClosestMapPoint run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;

                // startTime = ros::Time::now().toNSec();
                // optimizationICP();
                // endTime = ros::Time::now().toNSec();
                // cout << "The optimizationICP run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;

                initialized=false;
                double inoutsideRatio;
                if(numofInsidePoints!=0){
                    inoutsideRatio=(double)numofOutsidePoints/(double)numofInsidePoints;
                }
                else{
                    inoutsideRatio=10000;
                }
                std::cout<<"numofInsidePoints = "<<numofInsidePoints<<std::endl;
                std::cout<<"numofOutsidePoints = "<<numofOutsidePoints<<std::endl;

                std::cout<<"inoutsideRatio = "<<inoutsideRatio<<std::endl;

                rescueRoomStream<< mapHeader.stamp.toSec()<<","<<int(initialYawAngle+i*rescue_angle_interval)%360<<","
                <<corridorGuess[j](0)<<","<<corridorGuess[j](1)<<","<<robotPose(0,3)<<","<<robotPose(1,3)<<","<<inoutsideRatio<<","
                <<numofInsidePoints<<","<<insideScore<<","<<numofOutsidePoints<<","<<outsideScore<<","<<insideTotalRange<<endl;
                resetParameters();
                // if(pause_iter){
                //     while(1){
                //         std::cout<<"press enter to continue "<<std::endl;
                //         std::getchar();
                //         break;
                //     }
                // }
                endTime = ros::Time::now().toNSec();
                cout << "one guess run time is:" << (double)(endTime - startTime) / 1e6 << "ms" << endl;
            }
            double diff_angle=acos(robotPose(0,0))/3.14159*180-initialYawAngle;
            std::cout<<"-----------------------------------scoring, initial angle = "<<initialYawAngle+i*rescue_angle_interval<<std::endl;

            pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
        }
        // FIXME: get a single result of all assumptime.....
        if(!bTestRescue){
            initialized=true;
        }
        cout<<"end scoreParticles"<<endl;
    }
    // only use robot pose in params.yaml to initialize robot
    else{
    }

    // pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
    // initializedUsingMassCenter();
    
    // calClosestMapPoint();

    //optimization();
    // optimizationICP();
    // test();
    // resetParameters();
    rescueRoomStream.close();
    geometry_msgs::Pose pose;
    pubDONEsignal.publish(pose);
}

void CloudInitializer::checkingGuess(){
    insideScore=0;
    outsideScore=0;
    insideTotalRange=0;
    // numofInsidePoints=0;
    // numofOutsidePoints=0;

     pcl::PointCloud<pcl::PointXYZI>::Ptr  organizedCloudInRecord;
        organizedCloudInRecord.reset(new pcl::PointCloud<pcl::PointXYZI>());
        // *organizedCloudInRecord=*furthestRing;
        // 
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFurthestRing;
        downSizeFurthestRing.setLeafSize(scoreDownsampleRate, scoreDownsampleRate, scoreDownsampleRate); // for global map visualization
        downSizeFurthestRing.setInputCloud(furthestRing);
        downSizeFurthestRing.filter(*organizedCloudInRecord);
        cout<<"downsample size = "<<organizedCloudInRecord->points.size()<<endl;
    Eigen::Vector3f guessTemp;
    guessTemp << checkingGuessX,checkingGuessY,0;
    setInitialPose(checkingAngle,guessTemp);
    pcl::transformPointCloud(*organizedCloudInRecord,*transformed_pc,robotPose);
    pubPclCloud( transformed_pc, &pubTransformedPC, & mapHeader );
    cout<<"size of transformed pc"<<transformed_pc->points.size()<<endl;
    // calClosestMapPoint();
    initialized=false;
    resetParameters();
    cout << "checkingGuess........................." <<endl;
    //  if(pause_iter){
    //     while(1){
    //         std::cout<<"press enter to continue "<<std::endl;
    //         std::getchar();
    //         break;
    //     }
    // }
}

void CloudInitializer::allocateMemory(){
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    laserUppestRing.reset(new pcl::PointCloud<pcl::PointXYZI>());
    potentialCeilingPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    potentialCeilingPoints->points.resize(N_SCAN*Horizon_SCAN);

    organizedCloudIn.reset(new pcl::PointCloud<pcl::PointXYZI>());
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
    insidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    insidePC->points.resize(Horizon_SCAN);
    outsidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    outsidePC->points.resize(Horizon_SCAN);

    organizedCloudIn->points.resize(Horizon_SCAN);
    transformed_pc->points.resize(Horizon_SCAN);
    UsefulPoints1->points.resize(Horizon_SCAN);
    UsefulPoints2->points.resize(Horizon_SCAN);

    std::cout<<"allocate organize size= "<<organizedCloudIn->points.size()<<"intersectionOnMap size = "<<intersectionOnMap->points.size()<<std::endl;;
}

//called every frame and every guess
void CloudInitializer::resetParameters(){
    laserCloudIn->clear();
    UsefulPoints1->clear();
    UsefulPoints2->clear();
    potentialCeilingPoints->clear();
    potentialCeilingPoints->points.resize(N_SCAN*Horizon_SCAN);
    // laserUppestRing->clear();
    // potentialCeilingPoints->clear();
    transformed_pc->clear();
    // map_pc->clear();
    ringMapP1->clear();
    ringMapP1->points.resize(Horizon_SCAN);
    ringMapP2->clear();
    ringMapP2->points.resize(Horizon_SCAN);

    intersectionOnMap->clear();
    intersectionOnMap->points.resize(Horizon_SCAN);
    numIcpPoints=0;
    //FIXME:  only clear when not testRescue, what is rescue??????????
    // if(!bTestRescue){
    //     organizedCloudIn->clear();
    //     furthestRing->clear();
    //     furthestRing->points.resize(Horizon_SCAN);
    // }


    intersectionOnMap->clear();
    intersectionOnMap->points.resize(Horizon_SCAN);
    //
    insidePC->clear();
    outsidePC->clear();
    insidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    insidePC->points.resize(Horizon_SCAN);
    outsidePC.reset(new pcl::PointCloud<pcl::PointXYZI>());
    outsidePC->points.resize(Horizon_SCAN);
    // if(!initialized){
    //     //if not initialized, bRescueRobot&&bTestRescue behave same
    //     if(!bRescueRobot&&!bTestRescue){
    //         organizedCloudIn->points.resize(Horizon_SCAN);
    //         transformed_pc->points.resize(Horizon_SCAN);
    //         UsefulPoints1->points.resize(Horizon_SCAN);
    //         UsefulPoints2->points.resize(Horizon_SCAN);
    //     }
    //     else{
    //         organizedCloudIn->points.resize(N_SCAN*Horizon_SCAN);
    //         transformed_pc->points.resize(N_SCAN*Horizon_SCAN);
    //         UsefulPoints1->points.resize(N_SCAN*Horizon_SCAN);
    //         UsefulPoints2->points.resize(N_SCAN*Horizon_SCAN);
    //     }

    // }
    // else{
    //     if(!bTestRescue){
    //         organizedCloudIn->points.resize(N_SCAN*Horizon_SCAN);
    //         transformed_pc->points.resize(N_SCAN*Horizon_SCAN);
    //         UsefulPoints1->points.resize(N_SCAN*Horizon_SCAN);
    //         UsefulPoints2->points.resize(N_SCAN*Horizon_SCAN);
    //     }
    //     organizedCloudIn->points.resize(N_SCAN*Horizon_SCAN);
    //     transformed_pc->points.resize(N_SCAN*Horizon_SCAN);
    //     UsefulPoints1->points.resize(N_SCAN*Horizon_SCAN);
    //     UsefulPoints2->points.resize(N_SCAN*Horizon_SCAN);
        
    // }
}

double CloudInitializer::getScoreFromTwoPC(const Eigen::Matrix4f & robotPose, pcl::PointCloud<pcl::PointXYZI>::Ptr PC1,pcl::PointCloud<pcl::PointXYZI>::Ptr  PC2 ){
    return 0;
}
