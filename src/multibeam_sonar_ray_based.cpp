/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "ros/package.h"

#include <sensor_msgs/point_cloud2_iterator.h>

#include <functional>
#include <nps_uw_multibeam_sonar/multibeam_sonar_ray_based.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "gazebo/sensors/GpuRaySensor.hh"
#include "gazebo/rendering/GpuLaser.hh"

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(NpsGazeboRosMultibeamSonarRay)

/////////////////////////////////////////////////
NpsGazeboRosMultibeamSonarRay::NpsGazeboRosMultibeamSonarRay()
: SensorPlugin(), width(0), height(0)
{
  this->camera_info_connect_count_ = 0;
  this->point_cloud_connect_count_ = 0;
  this->last_camera_info_update_time_ = common::Time(0);
}

/////////////////////////////////////////////////
NpsGazeboRosMultibeamSonarRay::~NpsGazeboRosMultibeamSonarRay()
{
  this->newLaserFrameConnection.reset();

  this->parentSensor.reset();
  this->laserCamera.reset();
}

/////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonarRay::Load(sensors::SensorPtr _sensor,
                              sdf::ElementPtr _sdf)
{
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::GpuRaySensor>(_sensor);
  this->laserCamera = this->parentSensor->LaserCamera();

  if (!this->parentSensor)
  {
    gzerr << "NpsGazeboRosMultibeamSonarRay not attached to a GpuLaser sensor\n";
    return;
  }

  ROS_INFO_STREAM("====================0=================");
  if (!_sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ = "ray/camera_info";
  else
    this->camera_info_topic_name_ =
        _sdf->GetElement("cameraInfoTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudTopicName"))
    this->point_cloud_topic_name_ = "points";
  else
    this->point_cloud_topic_name_ =
        _sdf->GetElement("pointCloudTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudCutoff"))
    this->point_cloud_cutoff_ = 0.01;
  else
    this->point_cloud_cutoff_ =
        _sdf->GetElement("pointCloudCutoff")->Get<double>();

  ROS_INFO_STREAM("================1=====================");
  this->width = this->parentSensor->RangeCount();
  this->height = this->parentSensor->VerticalRangeCount();
  // this->width = this->laserCamera->ImageWidth();
  // this->height = this->laserCamera->ImageHeight();
  // this->depth = this->laserCamera->ImageDepth();
  // this->format = this->laserCamera->ImageFormat();

  ROS_INFO_STREAM(this->format);
  ROS_INFO_STREAM("=================2====================");
  this->newLaserFrameConnection =this->laserCamera->ConnectNewLaserFrame(
      std::bind(&NpsGazeboRosMultibeamSonarRay::OnNewLaserFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);

  ROS_INFO_STREAM("=================3====================");

  // copying from DepthCameraPlugin into GazeboRosCameraUtils
  // this->parentSensor_ = this->parentSensor;
  // this->width_ = this->width;
  // this->height_ = this->height;
  // this->depth_ = this->depth;
  // this->format_ = this->format;
  // this->camera_ = this->laserCamera;

  // this->load_connection_ =
  //   GazeboRosCameraUtils::OnLoad(
  //           boost::bind(&NpsGazeboRosMultibeamSonarRay::Advertise, this));
  // GazeboRosCameraUtils::Load(_sensor, _sdf);


  ROS_INFO_STREAM("Plugin Loaded!!!!!!!!!!!!");
}

// void NpsGazeboRosMultibeamSonarRay::Advertise()
// {
//   ROS_INFO_STREAM("=================3=========1===========");
//   ros::AdvertiseOptions camera_info_ao =
//     ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
//         this->camera_info_topic_name_, 1,
//         boost::bind(&NpsGazeboRosMultibeamSonarRay::CameraInfoConnect, this),
//         boost::bind(&NpsGazeboRosMultibeamSonarRay::CameraInfoDisconnect, this),
//         ros::VoidPtr(), &this->camera_queue_);
//   this->camera_info_pub_ =
//     this->rosnode_->advertise(camera_info_ao);

//   ROS_INFO_STREAM("=================3========2============");
//   ros::AdvertiseOptions point_cloud_ao =
//     ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
//       this->point_cloud_topic_name_, 1,
//       boost::bind(&NpsGazeboRosMultibeamSonarRay::PointCloudConnect, this),
//       boost::bind(&NpsGazeboRosMultibeamSonarRay::PointCloudDisconnect, this),
//       ros::VoidPtr(), &this->camera_queue_);
//   this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);
// }

// void NpsGazeboRosMultibeamSonarRay::PointCloudConnect()
// {
//   this->point_cloud_connect_count_++;
//   this->parentSensor->SetActive(true);
// }

// void NpsGazeboRosMultibeamSonarRay::PointCloudDisconnect()
// {
//   this->point_cloud_connect_count_--;
//   // if (this->point_cloud_connect_count_ <= 0)
//   //   this->parentSensor->SetActive(false);
// }

// void NpsGazeboRosMultibeamSonarRay::CameraInfoConnect()
// {
//   this->camera_info_connect_count_++;
// }

// void NpsGazeboRosMultibeamSonarRay::CameraInfoDisconnect()
// {
//   this->camera_info_connect_count_--;
// }

/////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonarRay::OnNewLaserFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format)
{

  ROS_INFO_STREAM("=================4====================");
  this->sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  ROS_INFO_STREAM("=================4=========1===========");
  if (this->parentSensor->IsActive())
  {
  ROS_INFO_STREAM("=================4=========2===========");
    // Deactivate if no subscribers
    if (this->point_cloud_connect_count_ <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
      ROS_INFO_STREAM("Ray sensor active");
      this->ComputePointCloud(_image);
    }
  }
  else
  {
  ROS_INFO_STREAM("=================4=========3===========");
    if (this->point_cloud_connect_count_ > 0)
      this->parentSensor->SetActive(true);
  }

  ROS_INFO_STREAM("=================5====================");
}

void NpsGazeboRosMultibeamSonarRay::ComputePointCloud(const float *_src)
{
  this->lock_.lock();

  this->point_cloud_msg_.header.frame_id
        = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec
        = this->sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nsec
        = this->sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step
        = this->point_cloud_msg_.point_step * this->width;

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg_);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(this->height * this->width);

  // resize if point cloud image to camera parameters if required
  this->point_cloud_image_.create(this->height, this->width, CV_32FC1);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");
  cv::MatIterator_<float> iter_image = this->point_cloud_image_.begin<float>();

  point_cloud_msg_.is_dense = true;

  float* toCopyFrom = const_cast<float*>(_src);
  int index = 0;

  double hfov = this->parentSensor->LaserCamera()->HFOV().Radian();
  double hfl = static_cast<double>(this->width) / (2.0 * tan(hfov/2.0));
  double vfov = this->parentSensor->LaserCamera()->VFOV().Radian();
  double vfl = static_cast<double>(this->height) / (2.0 * tan(vfov/2.0));

  for (uint32_t j = 0; j < this->height; j++)
  {
    double elevation;
    if (this->height > 1)
      elevation = atan2(static_cast<double>(j) -
                        0.5 * static_cast<double>(this->height-1), vfl);
    else
      elevation = 0.0;

    for (uint32_t i = 0; i < this->width;
         i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb, ++iter_image)
    {
      double azimuth;
      if (this->width > 1)
        azimuth = atan2(static_cast<double>(i) -
                        0.5 * static_cast<double>(this->width-1), hfl);
      else
        azimuth = 0.0;

      double depth = toCopyFrom[index++];

      // in optical frame hardcoded rotation
      // rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iter_x = depth * tan(azimuth);
      *iter_y = depth * tan(elevation);
      if (depth > this->point_cloud_cutoff_)
      {
        *iter_z = depth;
        *iter_image = sqrt(*iter_x * *iter_x +
                           *iter_y * *iter_y +
                           *iter_z * *iter_z);
      }
      else  // point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        *iter_image = 0.0;
        point_cloud_msg_.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = static_cast<uint8_t*>(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == this->height * this->width*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*this->width*3+0];
        iter_rgb[1] = image_src[i*3+j*this->width*3+1];
        iter_rgb[2] = image_src[i*3+j*this->width*3+2];
      }
      else if (this->image_msg_.data.size() == this->height * this->width)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*this->width];
        iter_rgb[1] = image_src[i+j*this->width];
        iter_rgb[2] = image_src[i+j*this->width];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }

  this->point_cloud_pub_.publish(this->point_cloud_msg_);

  this->lock_.unlock();
}


// /////////////////////////////////////////////////
// void NpsGazeboRosMultibeamSonarRay::PublishCameraInfo()
// {

//   ROS_INFO_STREAM("================= PublishCameraInfo ====================");

//   ROS_DEBUG_NAMED("laser_camera",
//     "publishing default camera info, then laser camera info");
//   GazeboRosCameraUtils::PublishCameraInfo();

//   if (this->camera_info_connect_count_ > 0)
//   {
//     common::Time sensor_update_time
//           = this->parentSensor_->LastMeasurementTime();

//     this->sensor_update_time_ = sensor_update_time;
//     if (sensor_update_time
//           - this->last_camera_info_update_time_
//           >= this->update_period_)
//     {
//       this->PublishCameraInfo(this->camera_info_pub_);
//       // , sensor_update_time);
//       this->last_camera_info_update_time_ = sensor_update_time;
//     }
//   }
// }

}