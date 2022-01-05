#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/distortion_models.hpp"

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
            this->declare_parameter("namespace", "");
            this->declare_parameter("frame_id", "");

            this->declare_parameter("camera/width", 0);
            this->declare_parameter("camera/height", 0);

            this->declare_parameter("camera/fx", 0.0);
            this->declare_parameter("camera/fy", 0.0);
            this->declare_parameter("camera/cx", 0.0);
            this->declare_parameter("camera/cy", 0.0);
            this->declare_parameter("camera/k1", 0.0);
            this->declare_parameter("camera/k2", 0.0);
            this->declare_parameter("camera/p1", 0.0);
            this->declare_parameter("camera/p2", 0.0);
            this->declare_parameter("camera/r12", 0.0);
            this->declare_parameter("camera/r13", 0.0);
            this->declare_parameter("camera/r21", 0.0);
            this->declare_parameter("camera/r23", 0.0);
            this->declare_parameter("camera/r31", 0.0);
            this->declare_parameter("camera/r32", 0.0);
            this->declare_parameter("camera/fx_p", 0.0);
            this->declare_parameter("camera/fy_p", 0.0);
            this->declare_parameter("camera/cx_p", 0.0);
            this->declare_parameter("camera/cy_p", 0.0);
            this->declare_parameter("camera/Tx", 0.0);

            fx_ = this->get_parameter("camera/fx").as_double();
            fy_ = this->get_parameter("camera/fy").as_double();
            cx_ = this->get_parameter("camera/cx").as_double();
            cy_ = this->get_parameter("camera/cy").as_double();
            k1_ = this->get_parameter("camera/k1").as_double();
            k2_ = this->get_parameter("camera/k2").as_double();
            p1_ = this->get_parameter("camera/p1").as_double();
            p2_ = this->get_parameter("camera/p2").as_double();
            r12_ = this->get_parameter("camera/r12").as_double();
            r13_ = this->get_parameter("camera/r13").as_double();
            r21_ = this->get_parameter("camera/r21").as_double();
            r23_ = this->get_parameter("camera/r23").as_double();
            r31_ = this->get_parameter("camera/r31").as_double();
            r32_ = this->get_parameter("camera/r32").as_double();
            fx_p_ = this->get_parameter("camera/fx_p").as_double();
            fy_p_ = this->get_parameter("camera/fy_p").as_double();
            cx_p_ = this->get_parameter("camera/cx_p").as_double();
            cy_p_ = this->get_parameter("camera/cy_p").as_double();
            Tx_ = this->get_parameter("camera/Tx").as_double();

            width_ = this->get_parameter("camera/width").as_int();
            height_ = this->get_parameter("camera/height").as_int();

            framerate_ = this->get_parameter("framerate").as_int();
            int period = (int) 1000.0 / framerate_;

            int sensor_id = this->get_parameter("sensor_id").as_int();

            namespace_ = this->get_parameter("namespace").as_string();
            frame_id_ = this->get_parameter("frame_id").as_string();
            
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(namespace_ + "/image_raw", 2);
            image_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(namespace_ + "/camera_info", 2);

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
            rclcpp::Time timestamp = this->get_clock()->now();
            image_msg->header.stamp = timestamp;
            image_msg->header.frame_id = frame_id_;
            image_msg->height = frame.rows;
            image_msg->width = frame.cols;
            image_msg->encoding = mat_type2encoding(frame.type());
            image_msg->is_bigendian = false;
            image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            image_msg->data.assign(frame.datastart, frame.dataend);

            auto cam_info_msg = sensor_msgs::msg::CameraInfo();
            cam_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            // Distortion matrix
            cam_info_msg.d.resize(5);
            cam_info_msg.d[0] = k1_;
            cam_info_msg.d[1] = k2_;
            cam_info_msg.d[2] = p1_;
            cam_info_msg.d[3] = p2_;
            cam_info_msg.d[4] = 0.0;

            // Intrinsic matrix
            cam_info_msg.k.fill(0.0);
            cam_info_msg.k[0] = fx_;
            cam_info_msg.k[2] = cx_;
            cam_info_msg.k[4] = fy_;
            cam_info_msg.k[5] = cy_;
            cam_info_msg.k[8] = 1.0;

            // Rectification matrix
            cam_info_msg.r.fill(0.0);

            for (size_t i = 0; i < 3; i++) {
                // Identity
                cam_info_msg.r[i + i * 3] = 1;
            }

            cam_info_msg.r[1] = r12_;
            cam_info_msg.r[2] = r13_;
            cam_info_msg.r[3] = r21_;
            cam_info_msg.r[5] = r23_;
            cam_info_msg.r[6] = r31_;
            cam_info_msg.r[7] = r32_;

            // Projection matrix
            cam_info_msg.p.fill(0.0);
            cam_info_msg.p[0] = fx_p_;
            cam_info_msg.p[2] = cx_p_;
            cam_info_msg.p[5] = fy_p_;
            cam_info_msg.p[6] = cy_p_;
            cam_info_msg.p[3] = Tx_;
            cam_info_msg.p[10] = 1.0;
            
            cam_info_msg.width = width_;
            cam_info_msg.height = height_;

            cam_info_msg.header.frame_id = frame_id_;
            cam_info_msg.header.stamp = timestamp;

            image_info_pub_->publish(cam_info_msg);
            image_pub_->publish(std::move(image_msg));
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr image_info_pub_;
        VideoCapture cam_;

        int framerate_;
        std::string namespace_;

        double fx_;
        double fy_;
        double cx_;
        double cy_;

        double k1_;
        double k2_;
        double p1_;
        double p2_;

        double r12_;
        double r13_;
        double r21_;
        double r23_;
        double r31_;
        double r32_;

        double fx_p_;
        double cx_p_;
        double fy_p_;
        double cy_p_;
        double Tx_;

        int width_;
        int height_;

        std::string frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonoImagePublisher>());
    rclcpp::shutdown();
    return 0;
}