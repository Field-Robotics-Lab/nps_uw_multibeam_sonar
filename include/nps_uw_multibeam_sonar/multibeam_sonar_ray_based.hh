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

#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <string>

// gazebo stuff
#include <sdf/Param.hh>
#include <sdf/Element.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

// camera stuff
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

// ros messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

// dynamic reconfigure stuff
#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>

// OpenCV
#include <opencv2/core.hpp>

// boost stuff
#include <boost/thread/mutex.hpp>


namespace gazebo
{
  class NpsGazeboRosMultibeamSonarRay : public SensorPlugin, GazeboRosCameraUtils
  {
    public: NpsGazeboRosMultibeamSonarRay();

    public: ~NpsGazeboRosMultibeamSonarRay();

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewLaserFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

    protected: unsigned int width, height, depth;
    protected: std::string format;

    protected: sensors::GpuRaySensorPtr parentSensor;
    protected: rendering::GpuLaserPtr laserCamera;

    private: event::ConnectionPtr newLaserFrameConnection;

    private: event::ConnectionPtr load_connection_;

    // overload with our own
    private: common::Time sensor_update_time_;
    protected: ros::Publisher camera_info_pub_;

    /// \brief Advertise
    public: virtual void Advertise();

    /// \brief Keep track of number of connctions for plugin outputs
    private: int point_cloud_connect_count_;
    private: int camera_info_connect_count_;
    private: void PointCloudConnect();
    private: void PointCloudDisconnect();
    private: void CameraInfoConnect();
    private: void CameraInfoDisconnect();

    /// \brief Compute a normal texture and implement sonar model
    private: void ComputePointCloud(const float *_src);
    private: sensor_msgs::PointCloud2 point_cloud_msg_;
    private: cv::Mat point_cloud_image_;
    private: std::string point_cloud_topic_name_;
    private: ros::Publisher point_cloud_pub_;
    private: double point_cloud_cutoff_;

    private: common::Time last_camera_info_update_time_;
    private: std::string camera_info_topic_name_;

    using GazeboRosCameraUtils::PublishCameraInfo;
    protected: virtual void PublishCameraInfo();

  };
}
#endif