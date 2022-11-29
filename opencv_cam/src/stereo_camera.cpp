#include "rclcpp/rclcpp.hpp"
#include "opencv_cam/opencv_cam_node.hpp"
#include <iostream>
#include "camera_calibration_parsers/parse.hpp"
#include <opencv2/opencv.hpp>

std::string gstreamer_pipeline (int sensor_id, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

class Hansu_cam : pubic rclcpp::Node
{
pubulic:
    Hansu_cam();
    : Node("hansu_cam"), count_(0)
    {
        assert(!cxt_.camera_info_path_.empty()); // readCalibration will crash if file_name is ""
        std::string camera_name1;
        if (camera_calibration_parsers::readCalibration(cxt_.camera_info_path_, camera_name1, camera_info_msg_1)) {
          RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
          camera_info_msg_1.header.frame_id = cxt_.camera_frame_id_;
          camera_info_pub_1 = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        } else {
          RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
          camera_info_pub_1 = nullptr;
        }

        image_pub_1 = create_publisher<sensor_msgs::msg::Image>("image_raw1", 10);
        
        assert(!cxt_.camera_info_path_.empty()); // readCalibration will crash if file_name is ""
        std::string camera_name2;
        if (camera_calibration_parsers::readCalibration(cxt_.camera_info_path_, camera_name2, camera_info_msg_2)) {
          RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
          camera_info_msg_2.header.frame_id = cxt_.camera_frame_id_;
          camera_info_pub_2 = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        } else {
          RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
          camera_info_pub_2 = nullptr;
        }
    }
           
} 

int main(int argc, char * argv[])
{

    rclcpp::init(argc,argv);
    auto node = std::make_shared<Hansu_cam>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
