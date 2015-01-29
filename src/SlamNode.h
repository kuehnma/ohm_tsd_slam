/*
 * SlamNode.h
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#ifndef SLAMNODE_H_
#define SLAMNODE_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <vector>

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obcore/base/Logger.h"

#include <boost/thread.hpp>

#define INIT_PSHS 1      //number of initial pushes into the grid
#define LAS_OFFS_X -0.19 //offset of the laser scanner to the base footprint

namespace ohm_tsd_slam
{
class Localization;
class ThreadSLAM;
class ThreadMapping;
class ThreadGrid;

/**
 * @class SlamNode
 * @brief Main node management of the 2D SLAM
 * @author Philipp Koch
 */
class SlamNode
{
public:

  /**
   * Default constructor
   */


  SlamNode(const std::string& content, obvious::EnumTsdGridLoadSource source = obvious::FILE);

  /**
   * Destructor
   */
  virtual ~SlamNode();

  /**
   * start
   * Method to start the SLAM
   */
  void start(void);

private:

  /**
   * initialize
   * Method to initialize the necessary parameters with the first received scan
   * @param initScan Initial scan
   */
  void initialize(const sensor_msgs::LaserScan& initScan);

  /**
   * run
   * Main SLAM method
   */
  void run(void);

  void localizeOnly(void);

  /**
   * laserScanCallBack
   * Callback method to laser subscriber
   * @param scan Laser scan
   */
  void laserScanCallBack(const sensor_msgs::LaserScan& scan);

  bool storeMapServiceCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * Main node handle
   */
  ros::NodeHandle _nh;

  /**
   * Laser subscriber
   */
  ros::Subscriber _laserSubs;

  ros::ServiceServer _storeMapServer;

  /**
   * Initilized flag
   */
  bool _initialized;

  /**
   * Representation
   */
  obvious::TsdGrid* _grid;

  /**
   * obvious::Sensor instance containing data and pose
   */
  obvious::SensorPolar2D* _sensor;

  /**
   * Localization instance
   */
  Localization* _localizer;

  /**
   * Mapping thread instance
   */
  ThreadMapping* _threadMapping;

  /**
   * Grid thread instance
   */
  ThreadGrid* _threadGrid;

  /**
   * Publishing mutex
   */
  boost::mutex _pubMutex;

  /**
   * X starting offset factor
   */
  double _xOffFactor;

  /**
   * Y starting offset factor
   */
  double _yOffFactor;

  /**
   * Starting yaw angle
   */
  double _yawOffset;

  /**
   * Minimum range threshold
   */
  double _minRange;

  /**
   * Maximum range threshold
   */
  double _maxRange;

  /**
   * Maximum range threshold
   */
  double _lowReflectivityRange;


  /**
   * Time interval between occupancy grid
   */
  double _gridPublishInterval;

  /**
   * Desired loop rate
   */
  double _loopRate;

  double _footPrintWidth;

  double _footPrintHeight;

  bool _localizeOnly;
};

} /* namespace ohm_tsdSlam */

#endif /* SLAMNODE_H_ */
