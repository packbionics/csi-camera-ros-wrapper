#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using namespace cv;

std::string
mat_type2encoding(int mat_type)
{
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

class MonoImagePublisher : public rclcpp::Node
{
    public:
        MonoImagePublisher()
        : Node("mono_image_publisher")
        {
            this->declare_parameter("framerate", 0);
            this->declare_parameter("sensor_id", 0);
            this->declare_parameter("topic", "");
            this->declare_parameter("frame_id", "");

            framerate_ = this->get_parameter("framerate").as_int();
            int period = (int) 1000.0 / framerate_;

            int sensor_id = this->get_parameter("sensor_id").as_int();

            topic_ = this->get_parameter("topic").as_string();
            
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, 2);
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period), std::bind(&MonoImagePublisher::timer_callback, this));
            
            std::string cmd0 = "nvarguscamerasrc sensor-id=";
            std::string cmd1 = " ! video/x-raw(memory:NVMM), width=3264, height=2464, format=(string)NV12, framerate=(fraction)";
            std::string cmd2 = "/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

            cam_.open(cmd0 + std::to_string(sensor_id) + cmd1 + std::to_string(framerate_) + cmd2, cv::CAP_GSTREAMER);
        }

    private:
        void timer_callback()
        { 
            Mat frame;
            cam_.read(frame);

            sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

            // Convert OpenCV Mat to ROS Image
            image_msg->header.stamp = this->get_clock()->now();
            image_msg->header.frame_id = this->get_parameter("frame_id").as_string();
            image_msg->height = frame.rows;
            image_msg->width = frame.cols;
            image_msg->encoding = mat_type2encoding(frame.type());
            image_msg->is_bigendian = false;
            image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            image_msg->data.assign(frame.datastart, frame.dataend);

            image_pub_->publish(std::move(image_msg));
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        VideoCapture cam_;

        int framerate_;
        std::string topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonoImagePublisher>());
    rclcpp::shutdown();
    return 0;
}