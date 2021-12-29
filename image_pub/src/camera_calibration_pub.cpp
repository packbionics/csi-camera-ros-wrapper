#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"

using namespace std::chrono_literals;


class CameraCalibrationPub : public rclcpp::Node
{
    public:
        CameraCalibrationPub()
        : Node("camera_info_publisher")
        {

            this->declare_parameter("Camera/fx", 0.0);
            this->declare_parameter("Camera/fy", 0.0);
            this->declare_parameter("Camera/cx", 0.0);
            this->declare_parameter("Camera/cy", 0.0);
            this->declare_parameter("Camera/k1", 0.0);
            this->declare_parameter("Camera/k2", 0.0);
            this->declare_parameter("Camera/p1", 0.0);
            this->declare_parameter("Camera/p2", 0.0);
            this->declare_parameter("Camera/width", 0);
            this->declare_parameter("Camera/height", 0);

            fx_ = this->get_parameter("Camera/fx").as_double();
            fy_ = this->get_parameter("Camera/fy").as_double();
            cx_ = this->get_parameter("Camera/cx").as_double();
            cy_ = this->get_parameter("Camera/cy").as_double();
            k1_ = this->get_parameter("Camera/k1").as_double();
            k2_ = this->get_parameter("Camera/k2").as_double();
            p1_ = this->get_parameter("Camera/p1").as_double();
            p2_ = this->get_parameter("Camera/p2").as_double();

            width_ = this->get_parameter("Camera/width").as_int();
            height_ = this->get_parameter("Camera/height").as_int();

            publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/left/camera_info",2);
            timer_ = this->create_wall_timer(
                1000ms, std::bind(&CameraCalibrationPub::publish_camera_info, this));
        }

    private:
        void publish_camera_info()
        {
            auto message = sensor_msgs::msg::CameraInfo();
            message.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            // Distortion matrix
            message.d.resize(5);
            message.d[0] = k1_;
            message.d[1] = k2_;
            message.d[2] = p1_;
            message.d[3] = p2_;
            message.d[4] = 0.0;

            // Intrinsic matrix
            message.k.fill(0.0);
            message.k[0] = fx_;
            message.k[2] = cx_;
            message.k[4] = fy_;
            message.k[5] = cy_;
            message.k[8] = 1.0;

            // Rectification matrix
            message.r.fill(0.0);

            for (size_t i = 0; i < 3; i++) {
                // Identity (no rectification)
                message.r[i + i * 3] = 1;
            }

            // Projection matrix
            message.p.fill(0.0);
            message.p[0] = fx_;
            message.p[2] = cx_;
            message.p[5] = fy_;
            message.p[6] = cy_;
            message.p[10] = 1.0;
            
            message.width = width_;
            message.height = height_;

            message.header.frame_id = "camera_link";

            publisher_->publish(message);
        }
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;

        double fx_;
        double fy_;
        double cx_;
        double cy_;

        double k1_;
        double k2_;
        double p1_;
        double p2_;

        int width_;
        int height_;

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCalibrationPub>());
    rclcpp::shutdown();
    return 0;
}