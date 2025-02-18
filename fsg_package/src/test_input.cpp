#include <memory>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"

class TestInput : public rclcpp::Node
{
  public:
    TestInput() : Node("test_input")
    {
      // set up image publisher
      image_transport::Publisher image_pub = image_transport::create_publisher(this, "fsg_input");

      // set up launch parameters
      double RESIZE_FACTOR;
      this->declare_parameter<double>("resize_factor", 0.25);
      this->get_parameter("resize_factor", RESIZE_FACTOR);
      int FRAME_WAIT_TIME_MS;
      this->declare_parameter<int>("frame_wait_time_ms", 50);
      this->get_parameter("frame_wait_time_ms", FRAME_WAIT_TIME_MS);
      std::string VIDEO_LOCATION;
      this->declare_parameter<std::string>("video_location", "testvid.mp4");
      this->get_parameter("video_location", VIDEO_LOCATION);

      // Setup video capture from file
      cv::VideoCapture cap(VIDEO_LOCATION);

      // Check if camera opened successfully
      if(!cap.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
      }
      
      while(1){
        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
          break;

        // Resize according to run time parameters
        cv::Mat img;
        cv::resize(frame, img, cv::Size(), RESIZE_FACTOR, RESIZE_FACTOR, cv::INTER_NEAREST);

        // publish image  
        auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        out_msg->header.frame_id = "map";
        image_pub.publish(out_msg);

        // wait time between frames
        cv::waitKey(FRAME_WAIT_TIME_MS);
      }
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestInput>());
  rclcpp::shutdown();
  return 0;
}
