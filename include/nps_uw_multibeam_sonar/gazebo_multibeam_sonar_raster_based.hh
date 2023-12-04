/*
 * Copyright 2020 Naval Postgraduate School
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

#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_HH

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ros messages stuff
#include <sensor_msgs/PointCloud2.h>
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

// camera stuff
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

// boost stuff
#include <boost/thread/mutex.hpp>

#include <opencv2/core.hpp>
#include <complex>
#include <valarray>
#include <sstream>
#include <chrono>
#include <string>

// gazebo stuff
#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>

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

  class NpsGazeboRosMultibeamSonar : public SensorPlugin, GazeboRosCameraUtils
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: NpsGazeboRosMultibeamSonar();

    /// \brief Destructor
    public: ~NpsGazeboRosMultibeamSonar();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Helper function to fill the list of fiducials with all models
    /// in the world if none are specified
    private: void PopulateFiducials();
    // From FiducialCameraPlugin
    /// \brief Selection buffer used for occlusion detection
    public: std::unique_ptr<rendering::SelectionBuffer> selectionBuffer;

    /// \brief Pointer to the scene.
    public: rendering::ScenePtr scene;

    /// \brief True to detect all objects in the world.
    public: bool detectAll = false;

    /// \brief A list of fiducials tracked by this camera.
    public: std::set<std::string> fiducials;

    /// \brief Advertise point cloud and depth image
    public: virtual void Advertise();

    /// \brief Update the controller
    protected: virtual void OnNewDepthFrame(const float *_image,
                                            unsigned int _width,
                                            unsigned int _height,
                                            unsigned int _depth,
                                            const std::string &_format);

    /// \brief Update the controller
    protected: virtual void OnNewImageFrame(const unsigned char *_image,
                                            unsigned int _width,
                                            unsigned int _height,
                                            unsigned int _depth,
                                            const std::string &_format);

    /// \brief Compute a normal texture and implement sonar model
    private: void ComputeSonarImage(const float *_src);
    private: void ComputePointCloud(const float *_src);
    private: double ComputeIncidence(double azimuth,
                                     double elevation,
                                     cv::Vec3f normal);
    private: cv::Mat ComputeNormalImage(cv::Mat& depth);
    private: void ComputeCorrector();
    private: cv::Mat rand_image;

    /// \brief Parameters for sonar properties
    private: double sonarFreq;
    private: double bandwidth;
    private: double soundSpeed;
    private: double maxDistance;
    private: double sourceLevel;
    private: bool constMu;
    private: bool customTag;
    private: double absorption;
    private: double attenuation;
    private: double verticalFOV;
    // constant reflectivity
    private: double mu;
    // variational reflectivity
    private: physics::WorldPtr world;
    private: std::string reflectivityDatabaseFileName;
    private: std::string reflectivityDatabaseFilePath;
    private: std::string customTagDatabaseFileName;
    private: std::string customTagDatabaseFilePath;
    private: std::vector<std::string> objectNames;
    private: std::vector<float> reflectivities;
    private: double biofouling_rating_coeff;
    private: double roughness_coeff;
    private: double maxDepth, maxDepth_before, maxDepth_beforebefore;
    private: double maxDepth_prev;
    private: bool calculateReflectivity;
    private: bool artificialVehicleVibration;
    private: cv::Mat reflectivityImage;
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
    private: float* elevation_angles;
    private: float plotScaler;
    private: float sensorGain;
    protected: bool debugFlag;

    /// \brief CSV log writing stream for verifications
    protected: std::ofstream writeLog;
    protected: u_int64_t writeCounter;
    protected: u_int64_t writeNumber;
    protected: u_int64_t writeInterval;
    protected: bool writeLogFlag;

    /// \brief Keep track of number of connctions for plugin outputs
    private: int depth_image_connect_count_;
    private: int depth_info_connect_count_;
    private: int point_cloud_connect_count_;
    private: int sonar_image_connect_count_;
    private: void DepthImageConnect();
    private: void DepthImageDisconnect();
    private: void DepthInfoConnect();
    private: void DepthInfoDisconnect();
    private: void NormalImageConnect();
    private: void NormalImageDisconnect();
    private: void PointCloudConnect();
    private: void PointCloudDisconnect();
    private: common::Time last_depth_image_camera_info_update_time_;

    /// \brief A pointer to the ROS node.
    /// A node will be instantiated if it does not exist.
    private: ros::Publisher depth_image_pub_;
    private: ros::Publisher normal_image_pub_;
    private: ros::Publisher point_cloud_pub_;
    private: ros::Publisher sonar_image_raw_pub_;
    private: ros::Publisher sonar_image_pub_;

    private: sensor_msgs::Image depth_image_msg_;
    private: sensor_msgs::Image normal_image_msg_;
    private: sensor_msgs::PointCloud2 point_cloud_msg_;
    private: marine_acoustic_msgs::ProjectedSonarImage sonar_image_raw_msg_;
    private: sensor_msgs::Image sonar_image_msg_;
    private: sensor_msgs::Image sonar_image_mono_msg_;
    private: cv::Mat point_cloud_image_;

    std::default_random_engine generator;

    using GazeboRosCameraUtils::PublishCameraInfo;
    protected: virtual void PublishCameraInfo();

    /// \brief image where each pixel contains the depth information
    private: std::string depth_image_topic_name_;
    private: std::string depth_image_camera_info_topic_name_;
    private: std::string point_cloud_topic_name_;
    private: std::string sonar_image_raw_topic_name_;
    private: std::string sonar_image_topic_name_;

    private: double point_cloud_cutoff_;

    // overload with our own
    private: common::Time depth_sensor_update_time_;
    protected: ros::Publisher depth_image_camera_info_pub_;

    private: event::ConnectionPtr load_connection_;

    // from DepthCameraPlugin
    protected: unsigned int width, height, depth;
    protected: std::string format;

    protected: sensors::DepthCameraSensorPtr parentSensor;
    protected: rendering::DepthCameraPtr depthCamera;

    private: event::ConnectionPtr newDepthFrameConnection;
    private: event::ConnectionPtr newImageFrameConnection;
    private: event::ConnectionPtr newRGBPointCloudConnection;

    // A couple of "convenience" functions for computing azimuth & elevation
    private: inline double Azimuth(int col)
    {
      double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
      double fl = static_cast<double>(this->width) / (2.0 * tan(hfov/2.0));
      double azimuth;
      if (this->width > 1)
        azimuth = atan2(static_cast<double>(col) -
                        0.5 * static_cast<double>(this->width-1), fl);
      else
        azimuth = 0.0;
      return azimuth;
    }

    private: inline double Elevation(int row)
    {
      double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
      double fl = static_cast<double>(this->width) / (2.0 * tan(hfov/2.0));
      double elevation;
      if (this->height > 1)
        elevation = atan2(static_cast<double>(row) -
                          0.5 * static_cast<double>(this->height-1), fl);
      else
        elevation = 0.0;
      return elevation;
    }
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
