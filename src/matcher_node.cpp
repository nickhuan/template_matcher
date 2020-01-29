#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <actionlib/server/simple_action_server.h>
#include <template_matcher/MatchTemplatesAction.h> 

static const std::string OPENCV_WINDOW = "Image window";

class TemplateMatcher
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  int template_id_;
  template_matcher::MatchTemplatesFeedback feedback_;
  template_matcher::MatchTemplatesResult result_;
//   image_transport::Publisher image_pub_;
  actionlib::SimpleActionServer<template_matcher::MatchTemplatesAction> action_server_;

  void mask(int id)
  {
    // cv::filter2D(src,dst,depth,mask);
  }

  double match(cv::Mat img, cv::Mat templ, int match_method = 0)
  {
    /// Do the Matching and Normalize
    cv::Mat result;
    /// Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    result.create( result_rows, result_cols, CV_32FC1 );    
    matchTemplate(img, templ, result, match_method );
    normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
  }

public:
  TemplateMatcher()
    : it_(nh_),
    action_server_(nh_, "match_templates", false)
  {

    //register the goal and feeback callbacks
    action_server_.registerGoalCallback(boost::bind(&TemplateMatcher::goalCB, this));
    action_server_.registerPreemptCallback(boost::bind(&TemplateMatcher::preemptCB, this));

    // Subscribe to input video feed from 3D camera
    image_sub_ = it_.subscribe("/ifm3d/camera/distance", 1, &TemplateMatcher::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // load params: address for template
    std::string template_addr_ = "";


    cv::namedWindow(OPENCV_WINDOW);
  }

  ~TemplateMatcher()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void goalCB()
  {
    template_id_ = action_server_.acceptNewGoal()->template_id;
  }

  void preemptCB()
  {
    action_server_.setPreempted();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    // 1. Conversion: copy image from ros sensor_msgs/Image to opencv cv::Mat format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); //const std::string 	TYPE_16UC1 = "16UC1" //mono16: CV_16UC1, 16-bit grayscale image
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // 2. Matching: Match with template

    // match(cv_ptr->image. template);

    // 3. Action server: publish feedback


    // // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "template_matcher");
  TemplateMatcher tm;
  ros::spin();
  return 0;
}