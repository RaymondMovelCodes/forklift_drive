/*
*****************************************************************************
* COPYRIGHT STATEMENT
* Copyright (c) 2024, Raymond Co.,Ltd. 
* All Rights Reserved.
* You can not use, copy or spread without official authorization.
*****************************************************************************
* Author: kuang
* Version: 1.0.1
* Date: 2025.09
* DESCRIPTION
* Raymond forklift drive module.
*/

#include "forklift_drive/forklift_drive.hpp"

using namespace driver;
std::string version = "1.0.1";

int main (int argc, char** argv)
{
  ros::init(argc, argv, "forklift_drive_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");
  ROS_INFO_STREAM("forklift drive startup, version: " << version);
  driver::ForkliftDrive dvr(node,private_nh);
  while(ros::ok() && dvr.Poll())
  {
    ros::spinOnce();
  }
  return 0;
}