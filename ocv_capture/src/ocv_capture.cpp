/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
//
// example of video capture (via OpenCV)
// based on gencam_cu by Nikolaus Correll
//
// http://code.google.com/p/robocraft-ros-pkg/
// http://robocraft.ru
//

#define OCV_CAP_OUTPUT_ENABLED 1

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

class OCVCaptureNode
{
public:
  ros::NodeHandle &node_;
  sensor_msgs::Image img_;
  
  ros::Time next_time_;
  int count_;
  
  IplImage* src;
  CvCapture* capture;
  
  image_transport::Publisher image_pub;
  
  OCVCaptureNode(ros::NodeHandle &node) : node_(node)
  {
    src=NULL;
    capture=NULL;
    
    image_transport::ImageTransport it(node);

    image_pub = it.advertise("camera/image_raw", 1);
    int camera_index = -1;
    ros::NodeHandle nh("~");
    nh.getParam("camera_index", camera_index);
    ROS_INFO("camera index: %d", camera_index);

    if(NULL==(capture = cvCaptureFromCAM(camera_index)))
    {
      ROS_ERROR("Error: cvCaptureFromCAM()");
      ROS_BREAK();
    }
    if(NULL==(src=cvQueryFrame(capture)))
    {
      ROS_ERROR("Error: cvQueryFrame()");
    }
    ROS_INFO("capture image: %d x %d", src->width, src->height);
    ROS_INFO("channels: %d", src->nChannels);
    ROS_INFO("depth: %d", src->depth);
    
#if OCV_CAP_OUTPUT_ENABLED
    cvNamedWindow("capture", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("capture", 50, 50);
#endif
    next_time_ = ros::Time::now();
    count_ = 0;
  }

  virtual ~OCVCaptureNode()
  {
    cvReleaseCapture(&capture);
#if OCV_CAP_OUTPUT_ENABLED
    cvDestroyWindow("capture");
#endif
  }

  bool take_and_send_image()
  {
    int key;
    if(NULL==(src=cvQueryFrame(capture)))
    {
      ROS_ERROR("Error: cvQueryFrame()");
      return false;
    }
    else
    {
#if OCV_CAP_OUTPUT_ENABLED
      cvShowImage("capture", src);
      key = cvWaitKey(10);
#endif
      sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(src, "bgr8");
      
      // publish
      image_pub.publish(msg);
      return true;
    }
  }

  bool spin()
  {
    while(node_.ok())
    {
      if(take_and_send_image())
      {
        ++count_;
        ros::Time now_time = ros::Time::now();
        if (now_time > next_time_)
        {
          ROS_INFO("FPS: %d", count_);
          count_ = 0;
          next_time_ = next_time_ + ros::Duration(1,0);
        }
      }
      else
      {
        ROS_ERROR("Couldn't take image!");
        usleep(1000000);
      }
      ros::spinOnce();
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ocv_capture");
  ros::NodeHandle n;
  OCVCaptureNode cam(n);
  cam.spin();
  return 0;
}
