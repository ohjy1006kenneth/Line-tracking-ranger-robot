#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class DepthCamera : public rclcpp::Node
{
public:
    DepthCamera() : Node("depth_camera")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_avoidance", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&DepthCamera::depth_image_callback, this, _1));

            

        
    }

private:
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        cv::Mat cv_image;

        try
        {
            cv_image = cv_bridge::toCvCopy(msg, "16UC1")->image;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }




        int center_x = 290;
        int center_y = 290;
        int region_size = 10;  // Define the size of the region to average around the center

        int min_x = std::max(0, center_x - region_size);
        int max_x = std::min(cv_image.cols - 1, center_x + region_size);
        int min_y = std::max(0, center_y - region_size);
        int max_y = std::min(cv_image.rows - 1, center_y + region_size);

        uint64_t depth_sum = 0;
        int count = 0;

        for (int y = min_y; y <= max_y; ++y)
        {
            for (int x = min_x; x <= max_x; ++x)
            {
                uint16_t depth = cv_image.at<uint16_t>(y, x);
                if (depth != 0)  // Only consider valid depth values
                {
                    depth_sum += depth;
                    count++;
                }
            }
        }

        uint16_t average_depth = count > 0 ? static_cast<uint16_t>(depth_sum / count) : 0;

        RCLCPP_INFO(this->get_logger(), "Depth at central region: %u mm", average_depth);

        std_msgs::msg::Bool output_msg;
        output_msg.data = (average_depth < 1000); //checks depth in mm 
        publisher_->publish(output_msg);
    }


    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCamera>());
    rclcpp::shutdown();
    return 0;
}