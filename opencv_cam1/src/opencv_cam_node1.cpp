#include "opencv_cam1/opencv_cam_node1.hpp"
#include <iostream>
#include "camera_calibration_parsers/parse.hpp"
#include <opencv2/opencv.hpp>

std::string gstreamer_pipeline (int sensor_id, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}


namespace opencv_cam1
{

  std::string mat_type2encoding(int mat_type)
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
        throw std::runtime_error("unsupported encoding type");
    }
  }

  OpencvCamNode1::OpencvCamNode1(const rclcpp::NodeOptions &options) :
    Node("opencv_cam1", options),
    canceled_(false)
  {
    RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

    // Initialize parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Register for parameter changed. NOTE at this point nothing is done when parameters change.
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Log the current parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
    CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "opencv_cam Parameters", OPENCV_CAM_ALL_PARAMS)

    // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), OPENCV_CAM_ALL_PARAMS)

    RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

    // Open file or device
      int sensor_id = 0 ; 
      int capture_width = 1280 ;
      int capture_height = 720 ;
      int display_width = 1280 ;
      int display_height = 720 ;
      int framerate = 30 ;
      int flip_method = 0 ;
      std::string pipeline1 = gstreamer_pipeline(
      sensor_id,
      capture_width,
      capture_height,
      display_width,
      display_height,
      framerate,
      flip_method);

      capture_ = std::make_shared<cv::VideoCapture>(pipeline1, cv::CAP_GSTREAMER);

      if (!capture_->isOpened()) {
        RCLCPP_ERROR(get_logger(), "cannot open camera ");
        return;
        publish_fps_ = framerate;

        double width = capture_width;
        double height = capture_height;
        next_stamp_ = now();
        capture_->set(cv::CAP_PROP_FRAME_HEIGHT, capture_height);
        capture_->set(cv::CAP_PROP_FRAME_WIDTH, capture_width);
        capture_->set(cv::CAP_PROP_FPS, framerate);
        RCLCPP_INFO(get_logger(), "device %d open, width %g, height %g, device fps %g",
                    0, width, height, framerate);
      }

      RCLCPP_INFO(get_logger(), "got camera info for 'stereo_camera_right'");
      camera_info_msg_.header.frame_id = "stereo_camera_right";
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);


      image_pub_1 = create_publisher<sensor_msgs::msg::Image>("image_raw1", 10);

      // Run loop on it's own thread
      thread_ = std::thread(std::bind(&OpencvCamNode1::loop, this));

      RCLCPP_INFO(get_logger(), "start publishing");
  }

  OpencvCamNode1::~OpencvCamNode1()
  {
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void OpencvCamNode1::validate_parameters()
  {}

  void OpencvCamNode1::loop()
  {
    cv::Mat frame1;

    while (rclcpp::ok() && !canceled_.load()) {
      // Read a frame, if this is a device block until a frame is available
      if (!capture_->read(frame1)) {
        RCLCPP_INFO(get_logger(), "EOF, stop publishing1");
        break;
      }
      auto stamp = now();

      // Avoid copying image message if possible
      sensor_msgs::msg::Image::UniquePtr image_msg1(new sensor_msgs::msg::Image());

      // Convert OpenCV Mat to ROS Image
      image_msg1->header.stamp = stamp;
      image_msg1->header.frame_id = "stereo_camera_right";
      image_msg1->height = frame1.rows;
      image_msg1->width = frame1.cols;
      image_msg1->encoding = mat_type2encoding(frame1.type());
      image_msg1->is_bigendian = false;
      image_msg1->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame1.step);
      image_msg1->data.assign(frame1.datastart, frame1.dataend);

      // Publish
      image_pub_1->publish(std::move(image_msg1));
      camera_info_msg_.header.stamp = stamp;
      camera_info_pub_->publish(camera_info_msg_);

      // Sleep if required
      if (cxt_.file_) {
        using namespace std::chrono_literals;
        next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000ns / publish_fps_};
        auto wait = next_stamp_ - stamp;
        if (wait.nanoseconds() > 0) {
          std::this_thread::sleep_for(static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
        }
      }
    }
  }

} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam1::OpencvCamNode1)
