/*
 * localize_node.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: phil
 */

#include <ros/ros.h>
#include "SlamNode.h"

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obcore/base/Logger.h"

#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localize_node");
  LOGMSG_CONF("slamlog.log", obvious::Logger::file_off|obvious::Logger::screen_on, DBG_DEBUG, DBG_ERROR);

  std::string dataPath;                     //toDo: integrate sevice call to hive
  bool source = false;
  ros::NodeHandle prvNh("~");
  prvNh.param<std::string>("data_source", dataPath, "/home/phil/.ros/tsd_grid.dat");
  ohm_tsd_slam::SlamNode localizeNode(dataPath, obvious::FILE);
  localizeNode.start();
}
