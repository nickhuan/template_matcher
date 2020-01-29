#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <template_matcher/MatchTemplatesAction.h> 
#include "opencv2/imgproc/imgproc.hpp"

class MatchingTemplates
{
private:
  actionlib::SimpleActionServer<template_matcher::MatchTemplatesAction> action_server_;

public:

  MatchingTemplates(ros::NodeHandle n)
    : action_server_(n, "match templates", false)
  {
    
  }


}