/*
 * ThreadLocalize.cpp
 *
 *  Created on: 28.10.2014
 *      Author: phil
 */

#include "ThreadLocalize.h"

#include "MultiSlamNode.h"
#include "ThreadMapping.h"
#include "Localization.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"

#include <boost/bind.hpp>

#include <cstring>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, boost::mutex* pubMutex, MultiSlamNode& parentNode, std::string nameSpace):
        _sensor(NULL),
        _newScan(false),
        _initialized(false)
{
  /**
   * width
   * height
   * namespace
   * laser topic
   * x offset
   * y offset
   * yaw offset
   * tf_base
   * tf_child
   */

  ros::NodeHandle prvNh("~");

  std::string poseParamServer;
  std::string poseTopic;
  poseParamServer = nameSpace + "/pose_topic";
  prvNh.param(poseParamServer, poseTopic, std::string("default_ns/pose"));
  //
  //  std::string tfBaseFrameId;
  //  prvNh.param("tf_base_frame", tfBaseFrameId, std::string("/map"));
  //
  //  std::string tfChildFrameId;
  //  std::string tfChildParamServer;
  //  tfChildParamServer = nameSpace + "/tf_child_frame";
  //  prvNh.param(tfChildParamServer, tfChildFrameId, std::string("default_ns/base_footprint"));

  std::string laserTopic;
  prvNh.param("laser_topic", laserTopic, std::string("scan"));
  laserTopic = nameSpace + "/" + laserTopic;

  std::string xOffParamServer;
  xOffParamServer = nameSpace + "/x_offset";
  prvNh.param<double>(xOffParamServer, _xOffset, 0.0);

  std::string yOffParamServer;
  yOffParamServer = nameSpace + "/y_offset";
  prvNh.param<double>(yOffParamServer, _yOffset, 0.0);

  std::string yawOffParamServer;
  yOffParamServer = nameSpace + "/yaw_offset";
  prvNh.param<double>(yOffParamServer, _yawOffset, 0.0);

  //  double sensorStaticXoffset = 0.0;
  //  std::string sensorStaticXoffsetParamServer;
  //  sensorStaticXoffsetParamServer = nameSpace + "sensor_static_offset_x";
  //  prvNh.param<double>(sensorStaticXoffsetParamServer, sensorStaticXoffset, -0.19);

  std::string maxRangeParamServer;
  maxRangeParamServer = nameSpace + "/max_range";
  prvNh.param<double>(maxRangeParamServer, _maxRange, 30.0);

  std::string minRangeParamServer;
  maxRangeParamServer = nameSpace + "/min_range";
  prvNh.param<double>(maxRangeParamServer, _minRange, 0.001);

  _gridWidth = grid->getCellsX() * grid->getCellSize();
  _gridHeight = grid->getCellsY() * grid->getCellSize();
  double _gridHeight;

  _xOffFactor = parentNode.xOffFactor();
  _yOffFactor = parentNode.yOffFactor();

  _localizer = new Localization(grid, mapper, pubMutex, parentNode, nameSpace);
  _sensor = NULL;
}

ThreadLocalize::~ThreadLocalize()
{
  delete _sensor;
  delete _localizer;
}


void ThreadLocalize::eventLoop(void)
{
  while(_stayActive)
  {
    ros::spinOnce();
    if(_newScan)
    {
      _localizer->localize(_sensor);
    }
    else
    {
      //maybe publish pose only ?
    }
    _newScan = false;
  }
}

void ThreadLocalize::init(const sensor_msgs::LaserScan& scan)
{
  //  _mask = new bool[scan.ranges.size()];
  //  for(unsigned int i=0; i < scan.ranges.size(); i++)
  //  {
  //    _mask[i] = !isnan(scan.ranges[i]) && !isinf(scan.ranges[i])&&(std::abs(scan.ranges[i])>10e-6);
  //  }

  _sensor=new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, static_cast<double>(_maxRange));
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  //_sensor->setRealMeasurementMask(_mask);

  double phi    = _yawOffset;
  double startX = _gridWidth*_xOffFactor; //toDo: add offset from this zero point from lauch
  double startY = _gridWidth*_yOffFactor;
  double tf[9]  = {std::cos(phi), -std::sin(phi), _gridWidth*_xOffFactor,
                   std::sin(phi),  std::cos(phi), _gridHeight*_yOffFactor,
                               0,              0,                      1};

  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);
  _sensor->transform(&Tinit);


}

void ThreadLocalize::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initailize node...\n";   //toDo: print ID of referring robot
    this->init(scan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
  }
//  for(unsigned int i=0;i<scan.ranges.size();i++)
//  {
//
//    _mask[i] = !isnan(scan.ranges[i]) && !isinf(scan.ranges[i]) && (fabs(scan.ranges[i]) > 10e-6);
//    if((_rangeFilter)&&_mask[i])
//      _mask[i]=(scan.ranges[i]>_minRange)&&(scan.ranges[i]<_maxRange);
//  }
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  //_sensor->setRealMeasurementMask(_mask);
  _newScan = true;
}

} /* namespace ohm_tsd_slam */