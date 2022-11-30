#ifndef OPENCV_CAM_HPP
#define OPENCV_CAM_HPP


#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv_cam1/camera_context.hpp"

namespace opencv_cam1
{

  class OpencvCamNode1 : public rclcpp::Node
  {
    CameraContext cxt_;

    std::thread thread_;
    std::atomic<bool> canceled_;

    std::shared_ptr<cv::VideoCapture> capture_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int publish_fps_;
    rclcpp::Time next_stamp_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_1;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  public:

    explicit OpencvCamNode1(const rclcpp::NodeOptions &options);

    ~OpencvCamNode1() override;

  private:

    void validate_parameters();

    void loop();
  };

} // namespace opencv_cam

#endif //OPENCV_CAM_HPP
