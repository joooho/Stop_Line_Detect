/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/

/**
 * @file simple_double.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

#include <aruco_msgs/multi_msg.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>
#include <vector>
#include <functional>
#include <stdio.h>

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages;
aruco::MarkerDetector mDetector;
std::vector<aruco::Marker> markers;
cv::Mat inImageNorm;

ros::Subscriber cam_info_sub;
bool cam_info_received;

image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;
ros::Publisher marker_pub;
ros::Time curr_stamp;

int hz;
double marker_size;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  
  double ticksBefore = cv::getTickCount();
  static tf::TransformBroadcaster br;

  curr_stamp = msg->header.stamp;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    inImage = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
  
void processing()
{
  if (!cam_info_received)
  {
      ROS_WARN("cam_info not received");
      return;
  }

  markers.clear();
  mDetector.detect(inImage, markers, camParam, marker_size, false);

  aruco_msgs::MarkerArray MarkerArrMsg;
  aruco_msgs::Marker MarkerMsg;
  geometry_msgs::Pose PoseMsg;

  for (std::size_t i = 0; i < markers.size(); ++i)
  {
      MarkerMsg.id = markers[i].id;
      tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
      tf::poseTFToMsg(transform, PoseMsg);
      MarkerMsg.pose.pose = PoseMsg;
      MarkerArrMsg.markers.push_back(MarkerMsg);
      markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);

      if (camParam.isValid() && marker_size != -1)
      {
         aruco::CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
      }
  }
  
  MarkerArrMsg.header.stamp = curr_stamp;
  MarkerArrMsg.header.frame_id = "aruco";
  marker_pub.publish(MarkerArrMsg);

  cv::circle(inImage, cv::Point(inImage.cols / 2, inImage.rows / 2), 4, cv::Scalar(0, 255, 0), 1);

  if (image_pub.getNumSubscribers() > 0)
  {
     cv_bridge::CvImage out_msg;
     out_msg.header.stamp = curr_stamp;
     out_msg.encoding = sensor_msgs::image_encodings::RGB8;
     out_msg.image = inImage;
     image_pub.publish(out_msg.toImageMsg());
  }

  if (debug_pub.getNumSubscribers() > 0)
  {
     cv_bridge::CvImage debug_msg;
     debug_msg.header.stamp = curr_stamp;
     debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
     debug_msg.image = mDetector.getThresholdedImage();
     debug_pub.publish(debug_msg.toImageMsg());
  }
}

void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  ROS_WARN("cam_info_callback %d",(int)cam_info_received);
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);
  nh.param<double>("marker_size", marker_size, 0.13);
  nh.param<int>("hz", hz, 10);
  ros::Rate loop_rate(hz);

  image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, &image_callback);
  cam_info_sub = nh.subscribe("/usb_cam/camera_info", 1, &cam_info_callback);
  cam_info_received = false;

  image_pub = it.advertise("result", 1);
  debug_pub = it.advertise("debug", 1);
  marker_pub = nh.advertise<aruco_msgs::MarkerArray>("markers", 1);

  while (ros::ok())
  {
    processing();
    loop_rate.sleep();
    ros::spinOnce();
  }
}
