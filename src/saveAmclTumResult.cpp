#include"utility.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
void amclResultCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    std::ofstream ofs;
    ofs.setf(ios::fixed);
    ofs.precision(6);
    ofs.open("/home/xiefujing/research/area_graph/ws/robotPoseResult/AmclTumResult.txt", ofstream::app);

    ofs << msg->header.stamp.toSec() << " " << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->pose.pose.position.z << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << " " << msg->pose.pose.orientation.w << std::endl;
}
int main(int argc, char** argv)
{

    ros::init(argc, argv, "saveAmclTumResult");

    ros::NodeHandle nh;
    // ros::Subscriber subAmclResult = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, boost::bind(&amclResultCallback,_1,ofs));
    ros::Subscriber subAmclResult = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &amclResultCallback);

    ros::spin();
    // ofs.close();
    
    return 0;
}