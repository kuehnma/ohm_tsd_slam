#include "Localization.h"
#include "SlamNode.h"
#include "ThreadMapping.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"

#include <boost/bind.hpp>

#include <cstring>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

Localization::Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, const double xOffFactor, const double yOffFactor):
                _gridOffSetX(-1.0 * grid->getCellsX() * grid->getCellSize() * xOffFactor),
                _gridOffSetY(-1.0 * grid->getCellsY() * grid->getCellSize() * yOffFactor)
{
  _mapper           = mapper;
  _grid             = grid;

  _scene            = NULL;
  _modelCoords      = NULL;
  _modelNormals     = NULL;

  _rayCaster        = new obvious::RayCastPolar2D();
  _assigner         = new obvious::FlannPairAssignment(2);
  _filterDist       = new obvious::DistanceFilter(2.0, 0.01, ITERATIONS - 3);
  _filterReciprocal = new obvious::ReciprocalFilter();
  _estimator        = new obvious::ClosedFormEstimator2D();
  _trnsMax          = TRNS_THRESH;   //toDo: config file
  _rotMax           = ROT_THRESH;    //toDo: config file
  _lastPose         = new obvious::Matrix(3, 3);
  _xOffFactor       = xOffFactor;
  _yOffFactor       = yOffFactor;

  //configure ICP
  _filterBounds     = new obvious::OutOfBoundsFilter2D(grid->getMinX(), grid->getMaxX(), grid->getMinY(), grid->getMaxY());
  _assigner->addPreFilter(_filterBounds);
  _assigner->addPostFilter(_filterDist);
  _assigner->addPostFilter(_filterReciprocal);
  _icp = new obvious::Icp(_assigner, _estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(ITERATIONS);
  _icp->setConvergenceCounter(ITERATIONS);

  std::string poseTopic;
  std::string tfBaseFrameId;
  std::string tfChildFrameId;
  ros::NodeHandle prvNh("~");
  prvNh.param             ("pose_topic", poseTopic, std::string("pose"));
  prvNh.param             ("tf_base_frame", tfBaseFrameId, std::string("/map"));
  prvNh.param             ("tf_child_frame", tfChildFrameId, std::string("laser"));
  prvNh.param<double>     ("sensor_static_offset_x", _lasXOffset, -0.19);
  prvNh.param<bool>       ("use_fused_tf", _useFusedPreTrafo, false);
  prvNh.param<std::string>("fused_tf_frame", _tfFrameFusedOdom, "odom_laser_combined");

  _posePub = nh.advertise<geometry_msgs::PoseStamped>(poseTopic, 1);

  _poseStamped.header.frame_id = tfBaseFrameId;
  _tf.frame_id_                = tfBaseFrameId;
  _tf.child_frame_id_          = tfChildFrameId;
}

Localization::~Localization()
{
  delete _rayCaster;
  delete _assigner;
  delete _filterBounds;
  delete _filterDist;
  delete _filterReciprocal;
  delete _estimator;
  delete _icp;
  delete _lastPose;
  if(_modelCoords)
    delete [] _modelCoords;
  if(_modelNormals)
    delete [] _modelNormals;
  if(_scene)
    delete [] _scene;
}

void Localization::localize(obvious::SensorPolar2D* sensor)
{
  unsigned int size = sensor->getRealMeasurementSize();

  if(!_scene)
  {
    _scene        = new double[size * 2];
    _modelCoords  = new double[size * 2];
    _modelNormals = new double[size * 2];
    *_lastPose    = sensor->getTransformation();
  }

  // reconstruction
  unsigned int modelSize = 0;
  _rayCaster->calcCoordsFromCurrentView(_grid, sensor, _modelCoords, _modelNormals, &modelSize);
  if(modelSize == 0)
  {
    std::cout << __PRETTY_FUNCTION__ << " Error! Raycasting found no coordinates!\n";
    return;
  }

  _icp->reset();
  obvious::Matrix P = sensor->getTransformation();
  _filterBounds->setPose(&P);

  obvious::Matrix M( modelSize / 2, 2, _modelCoords);
  obvious::Matrix N( modelSize / 2, 2, _modelNormals);
  _icp->setModel(&M, &N);

  size = sensor->dataToCartesianVector(_scene);
  obvious::Matrix S(size / 2, 2, _scene);
  _icp->setScene(&S);

  obvious::Matrix Tpre(4, 4);
  Tpre.setIdentity();
  if(_useFusedPreTrafo)
  {
    tf::StampedTransform odomFusedTf;
    odomFusedTf.frame_id_ = _tf.frame_id_;
    odomFusedTf.child_frame_id_ = _tfFrameFusedOdom;
    bool error = false;
    try
    {
      _tfListener.lookupTransform(_tf.frame_id_, _tfFrameFusedOdom, ros::Time(0), odomFusedTf);
    }
    catch(tf::TransformException& ex)
    {
      std::cout << __PRETTY_FUNCTION__ << " error looking up odom / slam fused tf" << std::endl;
      error = true;
    }
    if(!error)
    {
      tf::Matrix3x3 rotation = odomFusedTf.getBasis();
      for(unsigned int rows = 0; rows < 3; rows++)
      {
        tf::Vector3 row = rotation.getRow(rows);
        for(unsigned int cols = 0; cols < 3; cols++)
        {
          Tpre(rows, cols) = row[cols];
        }
      }
      tf::Vector3 t = odomFusedTf.getOrigin();
      for(unsigned int i = 0; i < 3; i++)
        Tpre(i, 3) = t[i];
      std::cout << __PRETTY_FUNCTION__ << "received rotatioin matrix\n";
      for(unsigned int rows = 0; rows < 3; rows++)
        std::cout << rotation.getRow(rows)[0] << " " << rotation.getRow(rows)[1] << " " << rotation.getRow(rows)[2] << std::endl;
      std::cout << "\nReceived t\n"
                << t[0] << " " << t[1] << " " << t[2] << "\n generated 4x4 matrix\n";
      Tpre.print();
      std::cout << std::endl;
    }
  }
  double rms = 0.0;
  unsigned int pairs = 0;
  unsigned int it = 0;
  _icp->iterate(&rms, &pairs, &it, &Tpre);
  obvious::Matrix T = _icp->getFinalTransformation();

  // analyze registration result
  double deltaX = T(0,2);
  double deltaY = T(1,2);
  double trnsAbs = std::sqrt(deltaX * deltaX + deltaY * deltaY);
  double deltaPhi = this->calcAngle(&T);
  _tf.stamp_ = ros::Time::now();

  if((trnsAbs > _trnsMax) || std::fabs(std::sin(deltaPhi)) > _rotMax)    //deltaY > 0.1 ||
  {
    // localization error broadcast invalid tf
    _poseStamped.header.stamp = ros::Time::now();
    _poseStamped.pose.position.x = NAN;
    _poseStamped.pose.position.y = NAN;
    _poseStamped.pose.position.z = NAN;
    tf::Quaternion quat(NAN, NAN, NAN);
    _poseStamped.pose.orientation.w = quat.w();
    _poseStamped.pose.orientation.x = quat.x();
    _poseStamped.pose.orientation.y = quat.y();
    _poseStamped.pose.orientation.z = quat.z();

    _tf.stamp_ = ros::Time::now();
    _tf.setOrigin(tf::Vector3(NAN, NAN, NAN));
    _tf.setRotation(quat);

    _posePub.publish(_poseStamped);
    _tfBroadcaster.sendTransform(_tf);
  }
  else            //transformation valid -> transform sensor
  {
    sensor->transform(&T);
    obvious::Matrix curPose = sensor->getTransformation();
    double curTheta = this->calcAngle(&curPose);
    double posX = curPose(0, 2) + _gridOffSetX;
    double posY = curPose(1, 2) + _gridOffSetY;
    _poseStamped.header.stamp    = ros::Time::now();
    _poseStamped.pose.position.x = posX;
    _poseStamped.pose.position.y = posY;
    _poseStamped.pose.position.z = 0.0;
    tf::Quaternion quat(0.0, 0.0, curTheta);
    _poseStamped.pose.orientation.w = quat.w();
    _poseStamped.pose.orientation.x = quat.x();
    _poseStamped.pose.orientation.y = quat.y();
    _poseStamped.pose.orientation.z = quat.z();

    _tf.stamp_ = ros::Time::now();
    _tf.setOrigin(tf::Vector3(posX, posY, 0.0));
    _tf.setRotation(quat);

    _posePub.publish(_poseStamped);
    _tfBroadcaster.sendTransform(_tf);
    if(_mapper)
    {
      if(this->isPoseChangeSignificant(_lastPose, &curPose))
      {
        *_lastPose = curPose;
        _mapper->queuePush(sensor);
      }
    }
  }
}

double Localization::calcAngle(obvious::Matrix* T)
{
  double angle          = 0.0;
  const double ARCSIN   = asin((*T)(1,0));
  const double ARCSINEG = asin((*T)(0,1));
  const double ARCOS    = acos((*T)(0,0));
  if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
    angle = ARCOS;
  else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
    angle = 2.0 * M_PI - ARCOS;
  return(angle);
}

bool Localization::isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose)
{
  double deltaX   = (*curPose)(0, 2) - (*lastPose)(0, 2);
  double deltaY   = (*curPose)(1, 2) - (*lastPose)(1, 2);
  double deltaPhi = this->calcAngle(curPose) - this->calcAngle(lastPose);
  deltaPhi        = fabs(sin(deltaPhi));
  double trnsAbs  = sqrt(deltaX * deltaX + deltaY * deltaY);

  return (deltaPhi > ROT_MIN) || (trnsAbs > TRNS_MIN);
}

} /* namespace */
