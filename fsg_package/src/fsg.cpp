#include <memory>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"

// start from FSG main a
#include <iostream>
#include <random>
#include "GreedyMerger.h"
#include "LsdOpenCV.h"
#include "Utils.h"

using namespace upm;

namespace cv {
inline void segments(cv::Mat img,
         const upm::Segments& segs,
         const cv::Scalar &color,
         int thickness = 1,
         int lineType = cv::LINE_8,
         int shift = 0) {
  for (const upm::Segment &seg: segs)
    cv::line(img, cv::Point2f(seg[0], seg[1]), cv::Point2f(seg[2], seg[3]), color, thickness, lineType, shift);
}
}

void drawClusters(cv::Mat &img,
             const upm::Segments &segs,
             const upm::SegmentClusters &clusters,
             int thickness = 2,
             cv::Scalar color = {0, 0, 0},
             bool drawLineCluster = true,
             int lineType = cv::LINE_AA,
             int shift = 0) {
  bool random_c = color == cv::Scalar(0, 0, 0);
  std::mt19937 mt(0);
  std::uniform_real_distribution<double> dist(1.0, 255.0);
  for (const std::vector<unsigned int> &cluster: clusters) {
    if (random_c) {
      color = cv::Scalar( dist(mt), dist(mt), dist(mt));
    }

    if (cluster.size() > 1 && drawLineCluster) {
      cv::Vec3d l = upm::totalLeastSquareFitSegmentEndPts(segs, cluster);

      cv::Point2f max_p(0, 0), min_p(img.cols, img.rows);
      for (unsigned int idx: cluster) {
        cv::Point2f pp;
        pp = upm::getProjectionPtn(l, cv::Point2f(segs[idx][0], segs[idx][1]));
        if (pp.x > max_p.x) max_p = pp;
        if (pp.x < min_p.x) min_p = pp;

        pp = upm::getProjectionPtn(l, cv::Point2f(segs[idx][2], segs[idx][3]));
        if (pp.x > max_p.x) max_p = pp;
        if (pp.x < min_p.x) min_p = pp;
      }
      cv::line(img, min_p, max_p, color, round(thickness / 3.0), lineType, shift);
    }
    for (unsigned int idx: cluster) {
      cv::segments(img, {segs[idx]}, color, thickness, lineType, shift);
    }
  }
}
// end from FSG main a

using std::placeholders::_1;

class FSG : public rclcpp::Node
{
  public:
    FSG()
    : Node("fsg")
    {
      // set up image subscriber
      auto custom_qos = rmw_qos_profile_system_default;
      custom_qos.depth = 1;
      image_sub = image_transport::create_subscription(this, "/fsg_input", std::bind(&FSG::imageCallback, this, std::placeholders::_1), "raw", custom_qos);

      // set up image publisher
      image_pub = image_transport::create_publisher(this, "fsg_output");

      // set up run time parameters
      this->declare_parameter<double>("resize_factor", 0.25);
      this->get_parameter("resize_factor", RESIZE_FACTOR);
    }

  private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
      // convert incoming image to opencv Mat
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat img;
      cv_ptr->image.copyTo(img);


      // Initialize the line segment merger
      GreedyMerger merger(img.size());

      //Detect lines
      upm::Segments detectedSegments;
      cv::Mat gray;
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
      cv::Mat img2 = img.clone();

      // Detect segments with LSD
      LsdOpenCV().detect(gray, detectedSegments);

      std::cout << "Detected " << detectedSegments.size() << " line segments with LSD" << std::endl;
      cv::segments(img, detectedSegments, CV_RGB(255, 0, 0), 1);
      cv::imshow("Detected line segments", img);
      //cv::imwrite("../Detected_line_segments.png", img);

      // Detect the segment clusters
      SegmentClusters detectedClusters;
      Segments mergedLines;
      merger.mergeSegments(detectedSegments, mergedLines, detectedClusters);
      drawClusters(img, detectedSegments, detectedClusters, 2);
      cv::imshow("Segment groups", img);
      //cv::imwrite("../Segment_groups.png", img);

      // Get large lines from groups of segments
      Segments filteredSegments, noisySegs;
      filterSegments(detectedSegments, detectedClusters, filteredSegments, noisySegs);
      cv::segments(img2, filteredSegments, CV_RGB(0, 255, 0));
      cv::segments(img2, noisySegs, CV_RGB(255, 0, 0));
      cv::imshow("Obtained lines", img2);
      //cv::imwrite("../Obtained_lines.png", img2);
      cv::waitKey(1);


      // publish image  
      auto out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
      out_msg->header.frame_id = "map";
      image_pub.publish(out_msg);
    }
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    double RESIZE_FACTOR;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FSG>());
  rclcpp::shutdown();
  return 0;
}
