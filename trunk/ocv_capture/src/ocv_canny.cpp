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
// example of Canny edge detection
//
// http://code.google.com/p/robocraft-ros-pkg/
// http://robocraft.ru
//
 
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  
  IplImage *src = 0, *gray = 0, *canny = 0;
  try
  {
    src = bridge.imgMsgToCv(msg, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  if(src)
  {
    //ROS_INFO("get image %d x %d", src->width, src->height);
    gray = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
    canny = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
    
    cvCvtColor(src, gray, CV_RGB2GRAY);
    cvCanny(gray, canny, 10, 100, 3);
    
    cvShowImage("canny", canny);
    
    cvReleaseImage(&gray);
    cvReleaseImage(&canny);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ocv_canny");
  ros::NodeHandle nh;
  cvNamedWindow("canny");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("canny");
} 
