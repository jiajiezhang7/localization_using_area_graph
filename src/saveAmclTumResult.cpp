#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <fstream>

class AmclResultSaver : public rclcpp::Node {
public:
    AmclResultSaver() : Node("amcl_result_saver") {
        // Initialize output file stream
        ofs_.setf(std::ios::fixed);
        ofs_.precision(6);
        ofs_.open("/home/xiefujing/research/area_graph/ws/robotPoseResult/AmclTumResult.txt", 
                  std::ofstream::app);

        if(!ofs_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file!");
            return;
        }

        // Create subscription
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&AmclResultSaver::amclResultCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "AmclResultSaver initialized");
    }

    ~AmclResultSaver() {
        if(ofs_.is_open()) {
            ofs_.close();
        }
    }

private:
    void amclResultCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if(!ofs_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Output file is not open!");
            return;
        }

        // Write pose data to file in TUM format
        ofs_ << rclcpp::Time(msg->header.stamp).seconds() << " "
             << msg->pose.pose.position.x << " "
             << msg->pose.pose.position.y << " "
             << msg->pose.pose.position.z << " "
             << msg->pose.pose.orientation.x << " "
             << msg->pose.pose.orientation.y << " "
             << msg->pose.pose.orientation.z << " "
             << msg->pose.pose.orientation.w << std::endl;

        RCLCPP_DEBUG(this->get_logger(), "Saved pose at time: %f", 
                     rclcpp::Time(msg->header.stamp).seconds());
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::ofstream ofs_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create and spin node
    auto node = std::make_shared<AmclResultSaver>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error during execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}