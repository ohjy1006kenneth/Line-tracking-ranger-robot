#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float64.hpp"

#include <iostream>

using namespace cv;
using namespace std;
using std::placeholders::_1;

#define RESULT_ROW 180
#define RESULT_COL 360
#define USED_ROW 180
#define USED_COL 360

class ColorCamera : public rclcpp::Node
{
public:
    ColorCamera() : Node("color_camera")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
        pid_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/pid_deviation", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&ColorCamera::color_image_callback, this, _1));
    }

private:
    /**
@param input image to be cropped.
Crop the image using the ROI, only keep the lower half
*/
    Mat crop_image(Mat input) const
    {
        Rect roi(0, 360, 1280, 360);
        Mat crpImg = input(roi);
        resize(crpImg, crpImg, Size(480, 360));
        return crpImg;
    }

    void reverse_perspective(Mat &input, Mat &output) const
    {
        // Define source points (these should be the corners of the region you want to transform)
        Point2f src_points[4];
        src_points[0] = Point2f(98, 23);  // top-left
        src_points[1] = Point2f(256, 23); // top-right
        src_points[2] = Point2f(74, 82);  // bottom-left
        src_points[3] = Point2f(280, 82); // bottom-right

        // Define destination points (these should be where you want the corners to be in the output image)
        Point2f dst_points[4];
        dst_points[0] = Point2f(135, 45);  // top-left
        dst_points[1] = Point2f(225, 45);  // top-right
        dst_points[2] = Point2f(135, 135); // bottom-left
        dst_points[3] = Point2f(225, 135); // bottom-right

        // Calculate the perspective transform matrix
        Mat transform_matrix = getPerspectiveTransform(src_points, dst_points);

        // Apply perspective transformation
        warpPerspective(input, output, transform_matrix, Size(RESULT_COL, RESULT_ROW));
    }

    Mat draw_image(Mat input) const
    {
        // Convert to grayscale
        Mat gray;
        cvtColor(input, gray, COLOR_BGR2GRAY);

        // Apply Gaussian blur to reduce noise and improve edge detection
        GaussianBlur(gray, gray, Size(7, 7), 0);
        threshold(gray, gray, 170, 255, THRESH_BINARY);

        // Edge detection
        Mat edges;
        Canny(gray, edges, 50, 150);

        // // Define region of interest (ROI)
        // Mat mask = Mat::zeros(edges.size(), edges.type());
        // Point points[1][4];
        // points[0][0] = Point(0, edges.rows);
        // points[0][1] = Point(edges.cols, edges.rows);
        // points[0][2] = Point(edges.cols, edges.rows);
        // points[0][3] = Point(edges.cols, edges.rows);
        // const Point *ppt[1] = {points[0]};
        // int npt[] = {4};
        // fillPoly(mask, ppt, npt, 1, Scalar(255, 255, 255), 8);

        // Mat masked_edges;
        // bitwise_and(edges, mask, masked_edges);

        // Detect lines using Hough transform
        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

        Point up, low;
        float upx = 0;
        float upy = 0;
        float lowx = 0;
        float lowy = 0;
        float max_len = 0;
        float err, slope;
        auto err_message = std_msgs::msg::Float64();

        // Draw left lines in green
        for (size_t i = 0; i < lines.size(); i++)
        {

            Vec4i l = lines[i];
            
            up = Point(l[0], l[1]);
            low = Point(l[2], l[3]);

            float length = (up.x - low.x) * (up.x - low.x) + (up.y - low.y) * (up.y - low.y);
            if(length > max_len) {
                max_len = length;
                upx = up.x;
                upy = up.y;
                lowx = low.x;
                lowy = low.y;
                slope = -((upy - lowy) / (upx - lowx));
            }
        }

        up.x = upx;
        low.x = lowx;

        line(input, Point(upx, upy), Point(lowx, lowy), Scalar(0, 255, 0), 3, LINE_AA);

        // line(input, up, low, Scalar(0, 255, 255), 3, LINE_AA);

        float middle_screen = input.cols / 2;
 

        err = -((up.x + low.x) / 2 - middle_screen);

        RCLCPP_INFO(this->get_logger(), "slope: %f\n", slope);

        if (slope < 0.0)
        {
            slope = -(slope);
        }

        if (err < 70 && err > -70)
        {
            err = 0.0;
        }

        if (slope <= 1.3 || err == 640.0 || err == -640.0)
        {
            err = 20000.0;
        }
        err_message.data = static_cast<double>(err);
        pid_publisher_->publish(err_message);

        RCLCPP_INFO(this->get_logger(), "error: %f\n", err);

        line(input, Point(input.cols / 2, 0), Point(input.cols / 2, input.rows), Scalar(0, 0, 255), 3, LINE_AA);

        return input;
    }

    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        Mat cv_image;

        try
        {
            cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        Mat corped_image, binary_image, rev_image, drawn_image;
        // corped_image = crop_image(cv_image);
        // binary_image = binarize_image(corped_image);
        //  reverse_perspective(binary_image, rev_image);
        drawn_image = draw_image(cv_image);

        sensor_msgs::msg::Image output_msg;
        std_msgs::msg::Header header;

        cv_bridge::CvImage(header, "bgr8", drawn_image).toImageMsg(output_msg);

        publisher_->publish(output_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pid_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorCamera>());
    rclcpp::shutdown();
    return 0;
}