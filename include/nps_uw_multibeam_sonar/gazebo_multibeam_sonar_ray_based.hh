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
#include <complex>
#include <valarray>
#include <sstream>
#include <chrono>

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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <marine_acoustic_msgs/ProjectedSonarImage.h>
#include <marine_acoustic_msgs/SonarImageData.h>
#include <marine_acoustic_msgs/PingInfo.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

// dynamic reconfigure stuff
#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>

// OpenCV
#include <opencv2/core.hpp>

// boost stuff
#include <boost/thread/mutex.hpp>

// For variational reflectivity
#include <sdf/Element.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include "selection_buffer/SelectionBuffer.hh"


namespace gazebo
{
  typedef std::complex<float> Complex;
  typedef std::valarray<Complex> CArray;
  typedef std::valarray<CArray> CArray2D;

  typedef std::valarray<float> Array;
  typedef std::valarray<Array> Array2D;

  class NpsGazeboRosMultibeamSonarRay : public SensorPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: NpsGazeboRosMultibeamSonarRay();

    /// \brief Destructor
    public: ~NpsGazeboRosMultibeamSonarRay();

    /// \brief Load the plugin
    /// \param take in SDF root element
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

    /// \brief Advertise
    public: virtual void Advertise();

    /// \brief Keep track of number of connctions for plugin outputs
    private: int point_cloud_connect_count_;
    private: int sonar_image_connect_count_;
    private: void PointCloudConnect();
    private: void PointCloudDisconnect();
    private: void SonarImageConnect();
    private: void SonarImageDisconnect();

    /// \brief Compute a normal texture and implement sonar model
    private: void UpdatePointCloud(const sensor_msgs::PointCloud2ConstPtr& _msg);
    private: void ComputeSonarImage();
    private: cv::Mat ComputeNormalImage(cv::Mat& depth);
    private: double point_cloud_cutoff_;

    private: void ComputeCorrector();
    private: cv::Mat rand_image;

    /// \brief Parameters for sonar properties
    private: double sonarFreq;
    private: double bandwidth;
    private: double soundSpeed;
    private: double maxDistance;
    private: double sourceLevel;
    private: bool constMu;
    private: double absorption;
    private: double attenuation;
    private: double verticalFOV;
    // constant reflectivity
    private: double mu;
    private: bool calculateReflectivity;
    private: cv::Mat reflectivityImage;
    private: std::vector<float> azimuth_angles;
    private: float* elevation_angles;
    private: float* rangeVector;
    private: float* window;
    private: float** beamCorrector;
    private: float beamCorrectorSum;
    private: int nFreq;
    private: double df;
    private: int nBeams;
    private: int nRays;
    private: int beamSkips;
    private: int raySkips;
    private: int ray_nAzimuthRays;
    private: int ray_nElevationRays;
    private: float plotScaler;
    private: float sensorGain;
    protected: bool debugFlag;

    /// \brief A pointer to the ROS node.
    /// A node will be instantiated if it does not exist.
    private: ros::Publisher point_cloud_pub_;
    private: ros::Publisher normal_image_pub_;
    private: ros::Publisher sonar_image_raw_pub_;
    private: ros::Publisher sonar_image_pub_;

    /// \brief Subcriber to VelodyneGpuLaserPointCloud
    private: ros::Subscriber VelodyneGpuLaserPointCloud;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue pointCloudSubQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread pointCloudSubQueueThread;
    /// \brief ROS helper function that processes messages
    private: void pointCloudSubThread();

    private: sensor_msgs::PointCloud2 point_cloud_msg_;
    private: sensor_msgs::Image normal_image_msg_;
    private: marine_acoustic_msgs::ProjectedSonarImage sonar_image_raw_msg_;
    private: sensor_msgs::Image sonar_image_msg_;
    private: sensor_msgs::Image sonar_image_mono_msg_;
    private: cv::Mat point_cloud_image_;
    private: cv::Mat point_cloud_normal_image_;

    private: std::string point_cloud_topic_name_;
    private: std::string sonar_image_raw_topic_name_;
    private: std::string sonar_image_topic_name_;

    /// \brief CSV log writing stream for verifications
    protected: std::ofstream writeLog;
    protected: u_int64_t writeCounter;
    protected: u_int64_t writeNumber;
    protected: u_int64_t writeInterval;
    protected: bool writeLogFlag;

  };

  ///////////////////////////////////////////
  inline double unnormalized_sinc(double t)
  {
    try
    {
      double results = sin(t)/t;
      if (results != results)
        return 1.0;
      else
        return sin(t)/t;
    }catch(int expn)
    {
      return 1.0;
    }
  }

  /// \brief A class to store fiducial data
  class FiducialData
  {
    /// \brief Fiducial ID
    public: std::string id;

    /// \brief Center point of the fiducial in the image
    public: ignition::math::Vector2i pt;
  };
}
#endif