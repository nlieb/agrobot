
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/image_encodings.h>
#include "yolo_v2_class.hpp"    // imported functions from DLL
#include "opencv.hpp"
#include "yolo_detector.hpp"

#include <detector/RegionOfInterestArray.h>


ros::Subscriber start_sub;
ros::Publisher roi_pub, image_pub;
detector::RegionOfInterestArray roiArray;
YoloDetector yd("", ""); // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

bool convertToMat(const sensor_msgs::ImageConstPtr &frame,
                  cv::Mat &output) {
  cv_bridge::CvImagePtr cvPtr =
          cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::RGB8);
  output = cvPtr->image;
  return true;
}

void cameraCb(const sensor_msgs::ImageConstPtr &frame) {
  cv::Mat inputImage;
  convertToMat(frame, inputImage);

  std::vector<sensor_msgs::RegionOfInterest> roiArrayVec;
  detector::RegionOfInterestArray roiArray;

  try {
    std::vector<bbox_t> result_vec = yd.detect_img(inputImage);

    for(auto bbox : result_vec) {
      sensor_msgs::RegionOfInterest roi;

      roi.x_offset = bbox.x;
      roi.y_offset = bbox.y;
      roi.height = bbox.h;
      roi.width = bbox.w;

      roiArrayVec.push_back(roi);

    }

    inputImage = yd.draw_rois(inputImage, result_vec);
  }
  catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
  catch (...) { std::cerr << "unknown exception \n"; getchar(); }

  roiArray.rois = roiArrayVec;
  roi_pub.publish(roiArray);
  image_pub.publish(inputImage);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  start_sub = nh.subscribe("/bottom_cam/image_raw", 1, &cameraCb);
  roi_pub = nh.advertise<detector::RegionOfInterestArray>("/detector/rois", 1);
  image_pub = nh.advertise<cv::Mat>("/detector/image", 1);

  ros::spin();
  return 0;
}