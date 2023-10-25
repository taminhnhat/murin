#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
        cv::namedWindow(OPENCV_WINDOW);

        // std::string image_path = "/home/nhattm/dev-ws/src/murin/murin_stream/Lenna.png";
        // cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        // cv::imshow(OPENCV_WINDOW, img);
        // cv::waitKey(100);

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        subscriber = image_transport::create_subscription(this, "/camera/color/image_raw", std::bind(&MinimalPublisher::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
    }

private:
    const std::string OPENCV_WINDOW = "Image window";
    image_transport::Subscriber subscriber;
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            // const std::string encode_ = cv_bridge::getEncoding(msg->encoding);
            // std::cout << "encoing " << msg->encoding << std::endl;
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

        cv::Mat imgOut;
        cv::flip(cv_ptr->image, imgOut, 1);
        cv::imshow(OPENCV_WINDOW, imgOut);
        cv::waitKey(3);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}