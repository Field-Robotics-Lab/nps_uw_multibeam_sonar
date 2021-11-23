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

#include <assert.h>
#include <sys/stat.h>
#include <tf/tf.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>

#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <functional>
#include <nps_uw_multibeam_sonar/multibeam_sonar_ray_based.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "gazebo/sensors/GpuRaySensor.hh"
#include "gazebo/rendering/GpuLaser.hh"

#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>

#include <algorithm>
#include <string>
#include <vector>
#include <limits>

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(NpsGazeboRosMultibeamSonarRay)

/////////////////////////////////////////////////
NpsGazeboRosMultibeamSonarRay::NpsGazeboRosMultibeamSonarRay()
: SensorPlugin(), width(0), height(0)
{
  // this->camera_info_connect_count_ = 0;
  this->point_cloud_connect_count_ = 0;
  this->sonar_image_connect_count_ = 0;
  // this->last_camera_info_update_time_ = common::Time(0);

  // frame counter for variational reflectivity
  this->maxDepth_before = 0.0;
  this->maxDepth_beforebefore = 0.0;
  this->maxDepth_prev = 0.0;

  // for csv write logs
  this->writeCounter = 0;
  this->writeNumber = 1;
}

/////////////////////////////////////////////////
NpsGazeboRosMultibeamSonarRay::~NpsGazeboRosMultibeamSonarRay()
{
  this->newLaserFrameConnection.reset();

  this->parentSensor.reset();
  this->laserCamera.reset();

  // CSV log write stream close
  writeLog.close();
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
  // if (!_sdf->HasElement("cameraInfoTopicName"))
  //   this->camera_info_topic_name_ = "ray/camera_info";
  // else
  //   this->camera_info_topic_name_ =
  //       _sdf->GetElement("cameraInfoTopicName")->Get<std::string>();

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
  this->depth = this->laserCamera->ImageDepth();
  this->format = this->laserCamera->ImageFormat();

  ROS_INFO_STREAM(this->format);
  ROS_INFO_STREAM( this->depth);
  ROS_INFO_STREAM("=================2====================");
  this->newLaserFrameConnection = this->laserCamera->ConnectNewLaserFrame(
      std::bind(&NpsGazeboRosMultibeamSonarRay::OnNewLaserFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);

  ROS_INFO_STREAM("=================3====================");

  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->laserCamera;

  // sonar stuff
  if (!_sdf->HasElement("sonarImageRawTopicName"))
    this->sonar_image_raw_topic_name_ = "sonar_image_raw";
  else
    this->sonar_image_raw_topic_name_ =
      _sdf->GetElement("sonarImageRawTopicName")->Get<std::string>();
  if (!_sdf->HasElement("sonarImageTopicName"))
    this->sonar_image_topic_name_ = "sonar_image";
  else
    this->sonar_image_topic_name_ =
      _sdf->GetElement("sonarImageTopicName")->Get<std::string>();

  // Read sonar properties from model.sdf
  if (!_sdf->HasElement("verticalFOV"))
    this->verticalFOV = 10;  // Blueview P900 -> 10 degrees
  else
    this->verticalFOV =
      _sdf->GetElement("verticalFOV")->Get<double>();
  if (!_sdf->HasElement("sonarFreq"))
    this->sonarFreq = 900e3;  // Blueview P900 [Hz]
  else
    this->sonarFreq =
      _sdf->GetElement("sonarFreq")->Get<double>();
  if (!_sdf->HasElement("bandwidth"))
    this->bandwidth = 29.5e6;  // Blueview P900 [Hz]
  else
    this->bandwidth =
      _sdf->GetElement("bandwidth")->Get<double>();
  if (!_sdf->HasElement("soundSpeed"))
    this->soundSpeed = 1500;
  else
    this->soundSpeed =
      _sdf->GetElement("soundSpeed")->Get<double>();
  if (!_sdf->HasElement("maxDistance"))
    this->maxDistance = 60;
  else
    this->maxDistance =
      _sdf->GetElement("maxDistance")->Get<double>();
  if (!_sdf->HasElement("sourceLevel"))
    this->sourceLevel = 220;
  else
    this->sourceLevel =
      _sdf->GetElement("sourceLevel")->Get<double>();
  if (!_sdf->HasElement("constantReflectivity"))
    this->constMu = true;
  else
    this->constMu =
      _sdf->GetElement("constantReflectivity")->Get<bool>();
  if (!_sdf->HasElement("raySkips"))
    this->raySkips = 10;
  else
    this->raySkips =
      _sdf->GetElement("raySkips")->Get<int>();
  if (!_sdf->HasElement("plotScaler"))
    this->plotScaler = 10;
  else
    this->plotScaler =
      _sdf->GetElement("plotScaler")->Get<float>();
  if (!_sdf->HasElement("sensorGain"))
    this->sensorGain = 0.02;
  else
    this->sensorGain =
      _sdf->GetElement("sensorGain")->Get<float>();
  // Configure skips
  if (this->raySkips == 0) this->raySkips = 1;

  // --- Variational Reflectivity --- //
  // Read the variational reflectivity database file path from the SDF file
  if (!this->constMu)
  {
    if (!_sdf->HasElement("reflectivityDatabaseFile"))
    {
      this->reflectivityDatabaseFileName = "variationalReflectivityDatabase.csv";
    }
    else
    {
      this->reflectivityDatabaseFileName =
        _sdf->GetElement("reflectivityDatabaseFile")->Get<std::string>();
      GZ_ASSERT(!this->reflectivityDatabaseFileName.empty(),
        "Empty variational reflectivity database file name");
    }
  }

  this->mu = 1e-3;  // default constant mu

  this->reflectivityDatabaseFilePath =
    ros::package::getPath("nps_uw_multibeam_sonar")
        + "/worlds/" + this->reflectivityDatabaseFileName;

  // Read csv file
  std::ifstream csvFile; std::string line;
  csvFile.open(this->reflectivityDatabaseFilePath);
  // skip the 3 lines
  getline(csvFile, line); getline(csvFile, line); getline(csvFile, line);
  while (getline(csvFile, line))
  {
      if (line.empty())  // skip empty lines:
      {
          continue;
      }
      std::istringstream iss(line);
      std::string lineStream;
      std::string::size_type sz;
      std::vector <std::string> row;
      while (getline(iss, lineStream, ','))
      {
          row.push_back(lineStream);
      }
      this->objectNames.push_back(row[0]);
      this->reflectivities.push_back(stof(row[1], &sz));
  }

  // // From FiducialCameraPlugin
  // if (this->camera_)
  // {
  //   this->scene = this->camera_->GetScene();
  // }
  // if (!this->camera_ || !this->scene)
  // {
  //   gzerr << "SonarDummy failed to load. "
  //       << "Camera and/or Scene not found" << std::endl;
  // }
  // // load the fiducials
  // if (_sdf->HasElement("fiducial"))
  // {
  //   sdf::ElementPtr elem = _sdf->GetElement("fiducial");
  //   while (elem)
  //   {
  //     this->fiducials.insert(elem->Get<std::string>());
  //     elem = elem->GetNextElement("fiducial");
  //   }
  // }
  // else
  // {
  //   gzmsg << "No fiducials specified. All models will be tracked."
  //       << std::endl;
  //   this->detectAll = true;
  // }



  // Transmission path properties (typical model used here)
  // More sophisticated model by Francois-Garrison model is available
  this->absorption = 0.0354;  // [dB/m]
  this->attenuation = this->absorption*log(10)/20.0;

  // Range vector
  const float max_T = this->maxDistance*2.0/this->soundSpeed;
  float delta_f = 1.0/max_T;
  const float delta_t = 1.0/this->bandwidth;
  this->nFreq = ceil(this->bandwidth/delta_f);
  delta_f = this->bandwidth/this->nFreq;
  const int nTime = nFreq;
  this->rangeVector = new float[nTime];
  for (int i = 0; i < nTime; i++)
  {
    this->rangeVector[i] = delta_t*i*this->soundSpeed/2.0;
  }

  // FOV, Number of beams, number of rays are defined at model.sdf
  // Currently, this->width equals # of beams, and this->height equals # of rays
  // Each beam consists of (elevation,azimuth)=(this->height,1) rays
  // Beam patterns
  this->nBeams = this->width;
  this->nRays = this->height;
  this->ray_nElevationRays = this->height;
  this->ray_nAzimuthRays = 1;

  // Print sonar calculation settings
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("============   SONAR PLUGIN LOADED   =============");
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("Maximum view range  [m] = " << this->maxDistance);
  ROS_INFO_STREAM("Distance resolution [m] = " <<
                    this->soundSpeed*(1.0/(this->nFreq*delta_f)));
  ROS_INFO_STREAM("# of Beams = " << this->nBeams);
  ROS_INFO_STREAM("# of Rays / Beam (Elevation, Azimuth) = ("
      << ray_nElevationRays << ", " << ray_nAzimuthRays << ")");
  ROS_INFO_STREAM("Calculation skips (Elevation) = "
      << this->raySkips);
  ROS_INFO_STREAM("# of Time data / Beam = " << this->nFreq);
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("");

  // get writeLog Flag
  if (!_sdf->HasElement("writeLog"))
    this->writeLogFlag = false;
  else
  {
    this->writeLogFlag = _sdf->Get<bool>("writeLog");
    if (this->writeLogFlag)
    {
      if (_sdf->HasElement("writeFrameInterval"))
        this->writeInterval = _sdf->Get<int>("writeFrameInterval");
      else
        this->writeInterval = 10;
      ROS_INFO_STREAM("Raw data at " << "/tmp/SonarRawData_{numbers}.csv");
      ROS_INFO_STREAM("every " << this->writeInterval << " frames");
      ROS_INFO_STREAM("");

      struct stat buffer;
      std::string logfilename("/tmp/SonarRawData_000001.csv");
      if (stat (logfilename.c_str(), &buffer) == 0)
        system("rm /tmp/SonarRawData*.csv");
    }
  }

  // Get debug flag for computation time display
  if (!_sdf->HasElement("debugFlag"))
    this->debugFlag = false;
  else
    this->debugFlag =
      _sdf->GetElement("debugFlag")->Get<bool>();

  // -- Pre calculations for sonar -- //
  // rand number generator
  this->rand_image = cv::Mat(this->height, this->width, CV_32FC2);
  uint64 randN = static_cast<uint64>(std::rand());
  cv::theRNG().state = randN;
  cv::RNG rng = cv::theRNG();
  rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.0f);

  // Hamming window
  this->window = new float[this->nFreq];
  float windowSum = 0;
  for (size_t f = 0; f < this->nFreq; f++)
  {
    this->window[f] = 0.54 - 0.46 * cos(2.0*M_PI*(f+1)/this->nFreq);
    windowSum += pow(this->window[f], 2.0);
  }
  for (size_t f = 0; f < this->nFreq; f++)
    this->window[f] = this->window[f]/sqrt(windowSum);

  // Sonar corrector preallocation
  this->beamCorrector = new float*[nBeams];
  for (int i = 0; i < nBeams; i++)
      this->beamCorrector[i] = new float[nBeams];
  this->beamCorrectorSum = 0.0;

  this->load_connection_ =
    GazeboRosCameraUtils::OnLoad(
            boost::bind(&NpsGazeboRosMultibeamSonarRay::Advertise, this));
  GazeboRosCameraUtils::Load(_sensor, _sdf);


  ROS_INFO_STREAM("Plugin Loaded!!!!!!!!!!!!");
}

void NpsGazeboRosMultibeamSonarRay::PopulateFiducials()
{
  this->fiducials.clear();

  // Check all models for inclusion in the frustum.
  rendering::VisualPtr worldVis = this->scene->WorldVisual();
  for (unsigned int i = 0; i < worldVis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = worldVis->GetChild(i);
    if (childVis->GetType() == rendering::Visual::VT_MODEL)
      this->fiducials.insert(childVis->Name());
  }
}

void NpsGazeboRosMultibeamSonarRay::pointCloudSubThread()
{
  static const double timeout = 0.01;
  while (this->rosnode_->ok())
  {
    this->pointCloudSubQueue.callAvailable(ros::WallDuration(timeout));
  }
}


void NpsGazeboRosMultibeamSonarRay::Advertise()
{
  // ROS_INFO_STREAM("=================3=========1===========");
  // ros::AdvertiseOptions camera_info_ao =
  //   ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
  //       this->camera_info_topic_name_, 1,
  //       boost::bind(&NpsGazeboRosMultibeamSonarRay::CameraInfoConnect, this),
  //       boost::bind(&NpsGazeboRosMultibeamSonarRay::CameraInfoDisconnect, this),
  //       ros::VoidPtr(), &this->camera_queue_);
  // this->camera_info_pub_ =
  //   this->rosnode_->advertise(camera_info_ao);

  // ROS_INFO_STREAM("=================3========2============");
  // ros::AdvertiseOptions point_cloud_ao =
  //   ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
  //     this->point_cloud_topic_name_, 1,
  //     boost::bind(&NpsGazeboRosMultibeamSonarRay::PointCloudConnect, this),
  //     boost::bind(&NpsGazeboRosMultibeamSonarRay::PointCloudDisconnect, this),
  //     ros::VoidPtr(), &this->camera_queue_);
  // this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);


  // Velodyne Laser sensor point cloud subscriber
  // this->VelodyneGpuLaserPointCloud = this->rosnode_->subscribe<sensor_msgs::PointCloud2>
  //       (this->point_cloud_topic_name_, 10 ,boost::bind(&NpsGazeboRosMultibeamSonarRay::UpdatePointCloud, this, _1));


  ros::SubscribeOptions so =
  ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
      "/" + this->point_cloud_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonarRay::UpdatePointCloud, this, _1),
      ros::VoidPtr(), &this->pointCloudSubQueue);
  this->VelodyneGpuLaserPointCloud = this->rosnode_->subscribe(so);
  // Spin up the queue helper thread.
  this->pointCloudSubQueueThread = std::thread(std::bind(
      &NpsGazeboRosMultibeamSonarRay::pointCloudSubThread, this));

  // Publisher for sonar image
  ros::AdvertiseOptions sonar_image_raw_ao =
    ros::AdvertiseOptions::create<acoustic_msgs::SonarImage>(
      this->sonar_image_raw_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->sonar_image_raw_pub_ = this->rosnode_->advertise(sonar_image_raw_ao);

  ros::AdvertiseOptions sonar_image_ao =
    ros::AdvertiseOptions::create<sensor_msgs::Image>(
      this->sonar_image_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->sonar_image_pub_ = this->rosnode_->advertise(sonar_image_ao);
}

void NpsGazeboRosMultibeamSonarRay::PointCloudConnect()
{
  this->point_cloud_connect_count_++;
  this->parentSensor->SetActive(true);
}

void NpsGazeboRosMultibeamSonarRay::PointCloudDisconnect()
{
  this->point_cloud_connect_count_--;
  if (this->point_cloud_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

void NpsGazeboRosMultibeamSonarRay::SonarImageConnect()
{
  this->sonar_image_connect_count_++;
  this->parentSensor->SetActive(true);
}

void NpsGazeboRosMultibeamSonarRay::SonarImageDisconnect()
{
  this->sonar_image_connect_count_--;
  if (this->sonar_image_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

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

  // ROS_INFO_STREAM("=================4=========1===========");
  if (this->parentSensor->IsActive())
  {
  // ROS_INFO_STREAM("=================4=========2===========");

    // // TODO Currently no deactivation function alive
    // // // Deactivate if no subscribers
    // if (this->point_cloud_connect_count_ <= 0)
    // {
    //   this->parentSensor->SetActive(false);
    // }
    // else
    // {
    //   // this->ComputePointCloud(_image);

      if (this->sonar_image_connect_count_ > 0){
        this->ComputeSonarImage();
      }
    // }
  }
  else
  {
  // ROS_INFO_STREAM("=================4=========3===========");
    if (this->sonar_image_connect_count_ > 0)
      this->parentSensor->SetActive(true);
  }

  // ROS_INFO_STREAM("=================5====================");

  // // For variational reflectivity
  // if (!this->constMu)
  // {
  //   // Calculate only if the maxDepth from depth camera is changed and stabled
  //   double min; cv::minMaxLoc(this->point_cloud_image_, &min, &this->maxDepth);
  //   if (this->maxDepth == this->maxDepth_before
  //       && this->maxDepth == this->maxDepth_beforebefore
  //       && this->calculateReflectivity == false
  //       && this->maxDepth != this->maxDepth_prev)
  //   {
  //     this->calculateReflectivity = true;
  //     this->maxDepth_prev = this->maxDepth;

  //     // Regenerate reand image
  //     uint64 randN = static_cast<uint64>(std::rand());
  //     cv::theRNG().state = randN;
  //     cv::RNG rng = cv::theRNG();
  //     rng.fill(this->rand_image, cv::RNG::NORMAL, 0.f, 1.f);
  //   }
  //   else
  //     this->calculateReflectivity = false;

  //   this->maxDepth_beforebefore = this->maxDepth_before;
  //   this->maxDepth_before = this->maxDepth;

  //   if (calculateReflectivity)
  //   {
  //     // Generate reflectivity opencv image palette
  //     cv::Mat reflectivity_image = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

  //     if (!this->selectionBuffer)
  //     {
  //       std::string cameraName = this->camera_->OgreCamera()->getName();
  //       this->selectionBuffer.reset(
  //           new rendering::SelectionBuffer(cameraName,
  //           this->scene->OgreSceneManager(),
  //           this->camera_->RenderTexture()->getBuffer()->
  //           getRenderTarget()));
  //     }

  //     if (this->detectAll)
  //       this->PopulateFiducials();

  //     std::vector<FiducialData> results;
  //     for (const auto &f : this->fiducials)
  //     {
  //       // check if fiducial is visible within the frustum
  //       rendering::VisualPtr vis = this->scene->GetVisual(f);
  //       if (!vis)
  //         continue;

  //       if (!this->camera_->IsVisible(vis))
  //         continue;

  //       // Loop over every pixel
  //       for (int i=0; i<reflectivity_image.rows; i++)
  //       {
  //         for (int j=0; j<reflectivity_image.cols; j+=raySkips)
  //         {
  //           // target pixel
  //           ignition::math::Vector2i pt = ignition::math::Vector2i(i, j);

  //           // use selection buffer to check if visual is occluded by other entities
  //           // in the camera view
  //           Ogre::Entity *entity =
  //             this->selectionBuffer->OnSelectionClick(pt.X(), pt.Y());

  //           rendering::VisualPtr result;
  //           if (entity && !entity->getUserObjectBindings().getUserAny().isEmpty())
  //           {
  //             try
  //             {
  //               result = this->scene->GetVisual(
  //                   Ogre::any_cast<std::string>(
  //                   entity->getUserObjectBindings().getUserAny()));
  //             }
  //             catch(Ogre::Exception &_e)
  //             {
  //               gzerr << "Ogre Error:" << _e.getFullDescription() << "\n";
  //               continue;
  //             }
  //           }

  //           if (result && result->GetRootVisual() == vis)
  //           {
  //             FiducialData fd;
  //             fd.id = vis->Name();
  //             fd.pt = pt;

  //             // Assign variational reflectivity
  //             for (int k=0; k<objectNames.size(); k++)
  //               if (vis->Name() == objectNames[k])
  //                 reflectivity_image.at<float>(j, i) = reflectivities[k];
  //             // results.push_back(fd);  // Redundant
  //           }
  //         }
  //       }  // end of pixel loop
  //     }  // end of selection buffer

  //     // Save reflectivity image
  //     this->reflectivityImage = reflectivity_image;
  //   }  // end of variational reflectivity calculation
  // }  // end of variational reflectivity bool

}

/////////////////////////////////////////////////
// Most of the plugin work happens here
void NpsGazeboRosMultibeamSonarRay::ComputeSonarImage()
{
  this->lock_.lock();

  cv::Mat depth_image = this->point_cloud_image_;
  cv::Mat normal_image = this->ComputeNormalImage(depth_image);
  double vFOV = this->parentSensor->VertFOV();
  double hFOV = this->parentSensor->HorzFOV();
  double vPixelSize = vFOV / this->height;
  double hPixelSize = hFOV / this->width;

  if (this->beamCorrectorSum == 0)
    ComputeCorrector();

  // Default value for reflectivity
  if (this->reflectivityImage.rows == 0)
    this->reflectivityImage = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

ROS_INFO_STREAM("================= Ray based ==============");
ROS_INFO_STREAM(depth_image.size());
ROS_INFO_STREAM(normal_image.size());
ROS_INFO_STREAM(hPixelSize);
ROS_INFO_STREAM(vPixelSize);
ROS_INFO_STREAM(hFOV);
ROS_INFO_STREAM(vFOV);
ROS_INFO_STREAM(verticalFOV/180*M_PI);

  // For calc time measure
  auto start = std::chrono::high_resolution_clock::now();
  // ------------------------------------------------//
  // --------      Sonar calculations       -------- //
  // ------------------------------------------------//
  CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper(
                  depth_image,   // cv::Mat& depth_image
                  normal_image,  // cv::Mat& normal_image
                  rand_image,    // cv::Mat& rand_image
                  hPixelSize,    // hPixelSize
                  vPixelSize,    // vPixelSize
                  hFOV,          // hFOV
                  vFOV,          // VFOV
                  hPixelSize,    // _beam_azimuthAngleWidth
                  verticalFOV/180*M_PI,  // _beam_elevationAngleWidth
                  hPixelSize,    // _ray_azimuthAngleWidth
                  vPixelSize*(raySkips+1),  // _ray_elevationAngleWidth
                  this->soundSpeed,    // _soundSpeed
                  this->maxDistance,   // _maxDistance
                  this->sourceLevel,   // _sourceLevel
                  this->nBeams,        // _nBeams
                  this->nRays,         // _nRays
                  this->raySkips,      // _raySkips
                  this->sonarFreq,     // _sonarFreq
                  this->bandwidth,     // _bandwidth
                  this->nFreq,         // _nFreq
                  this->reflectivityImage,  // reflectivity_image
                  this->attenuation,   // _attenuation
                  this->window,        // _window
                  this->beamCorrector,      // _beamCorrector
                  this->beamCorrectorSum,   // _beamCorrectorSum
                  this->debugFlag);

  // For calc time measure
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<
                  std::chrono::microseconds>(stop - start);
  if (debugFlag)
  {
    ROS_INFO_STREAM("GPU Sonar Frame Calc Time " <<
                    duration.count()/10000 << "/100 [s]\n");
  }

  // Gaussian noise
  // double whiteNoise = ignition::math::Rand::DblNormal(0.0, 0.7);
      // ROS_INFO_STREAM(Intensity[beam][f]);

  // CSV log write stream
  // Each cols corresponds to each beams
  if (this->writeLogFlag)
  {
    this->writeCounter = this->writeCounter + 1;
    if (this->writeCounter == 1
        ||this->writeCounter % this->writeInterval == 0)
    {
      double time = this->parentSensor_->LastMeasurementTime().Double();
      std::stringstream filename;
      filename << "/tmp/SonarRawData_" << std::setw(6) <<  std::setfill('0')
               << this->writeNumber << ".csv";
      writeLog.open(filename.str().c_str(), std::ios_base::app);
      filename.clear();
      writeLog << "# Raw Sonar Data Log (Row: beams, Col: time series data)\n";
      writeLog << "# First column is range vector\n";
      writeLog << "#  nBeams : " << nBeams << "\n";
      writeLog << "# Simulation time : " << time << "\n";
      for (size_t i = 0; i < P_Beams[0].size(); i++)
      {
        // writing range vector at first column
        writeLog << this->rangeVector[i];
        for (size_t b = 0; b < nBeams; b ++)
        {
          if (P_Beams[b][i].imag() > 0)
            writeLog << "," << P_Beams[b][i].real()
                     << "+" << P_Beams[b][i].imag() << "i";
          else
            writeLog << "," << P_Beams[b][i].real()
                     << P_Beams[b][i].imag() << "i";
        }
        writeLog << "\n";
      }
      writeLog.close();

      this->writeNumber = this->writeNumber + 1;
    }
  }

  // Sonar image ROS msg
  this->sonar_image_raw_msg_.header.frame_id
        = this->frame_name_.c_str();
  this->sonar_image_raw_msg_.header.stamp.sec
        = this->sensor_update_time_.sec;
  this->sonar_image_raw_msg_.header.stamp.nsec
        = this->sensor_update_time_.nsec;
  this->sonar_image_raw_msg_.frequency = this->sonarFreq;
  this->sonar_image_raw_msg_.sound_speed = this->soundSpeed;
  this->sonar_image_raw_msg_.azimuth_beamwidth = hPixelSize;
  this->sonar_image_raw_msg_.elevation_beamwidth = hPixelSize*this->nRays;
  std::vector<float> azimuth_angles;
  double fl = static_cast<double>(width) / (2.0 * tan(hFOV/2.0));
  for (size_t beam = 0; beam < nBeams; beam ++)
    azimuth_angles.push_back(atan2(static_cast<double>(beam) -
                    0.5 * static_cast<double>(width-1), fl));
  this->sonar_image_raw_msg_.azimuth_angles = azimuth_angles;
  // std::vector<float> elevation_angles;
  // elevation_angles.push_back(vFOV / 2.0);  // 1D in elevation
  // this->sonar_image_raw_msg_.elevation_angles = elevation_angles;
  std::vector<float> ranges;
  for (size_t i = 0; i < P_Beams[0].size(); i ++)
    ranges.push_back(rangeVector[i]);
  this->sonar_image_raw_msg_.ranges = ranges;

  // this->sonar_image_raw_msg_.is_bigendian = false;
  this->sonar_image_raw_msg_.data_size = 1;  // sizeof(float) * nFreq * nBeams;
  std::vector<uchar> intensities;
  int Intensity[nBeams][nFreq];
  for (size_t f = 0; f < nFreq; f ++)
  {
    for (size_t beam = 0; beam < nBeams; beam ++)
    {
      Intensity[beam][f] = static_cast<int>(this->sensorGain * abs(P_Beams[beam][f]));
      uchar counts = static_cast<uchar>(std::min(UCHAR_MAX, Intensity[beam][f]));
      intensities.push_back(counts);
    }
  }
  this->sonar_image_raw_msg_.intensities = intensities;
  this->sonar_image_raw_pub_.publish(this->sonar_image_raw_msg_);

  // Construct visual sonar image for rqt plot in sensor::image msg format
  cv_bridge::CvImage img_bridge;

  // Generate image of 32FC1
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_32FC1);

  const float rangeMax = maxDistance;
  const float rangeRes = ranges[1]-ranges[0];
  const int nEffectiveRanges = ceil(rangeMax / rangeRes);
  const unsigned int radius = Intensity_image.size().height;
  const cv::Point origin(Intensity_image.size().width/2,
                         Intensity_image.size().height);
  const float binThickness = 2 * ceil(radius / nEffectiveRanges);

  struct BearingEntry
  {
    float begin, center, end;
    BearingEntry(float b, float c, float e)
      : begin(b), center(c), end(e)
        {;}
  };

  std::vector<BearingEntry> angles;
  angles.reserve(nBeams);

  for ( int b = 0; b < nBeams; ++b )
  {
    const float center = azimuth_angles[b];
    float begin = 0.0, end = 0.0;
    if (b == 0)
    {
      end = (azimuth_angles[b + 1] + center) / 2.0;
      begin = 2 * center - end;
    }
    else if (b == nBeams - 1)
    {
      begin = angles[b - 1].end;
      end = 2 * center - begin;
    }
    else
    {
      begin = angles[b - 1].end;
      end = (azimuth_angles[b + 1] + center) / 2.0;
    }
    angles.push_back(BearingEntry(begin, center, end));
  }

  const float ThetaShift = 1.5*M_PI;
  for ( int r = 0; r < ranges.size(); ++r )
  {
    if ( ranges[r] > rangeMax ) continue;
    for ( int b = 0; b < nBeams; ++b )
    {
      const float range = ranges[r];
      const int intensity = this->sensorGain * abs(P_Beams[b][r]);
      const float begin = angles[b].begin + ThetaShift,
                  end = angles[b].end + ThetaShift;
      const float rad = static_cast<float>(radius) * range/rangeMax;
      // Assume angles are in image frame x-right, y-down
      cv::ellipse(Intensity_image, origin, cv::Size(rad, rad), 0,
                  begin * 180/M_PI, end * 180/M_PI,
                  intensity/2500.0*this->plotScaler,
                  binThickness);
    }
  }

  // Publish final sonar image
  this->sonar_image_msg_.header.frame_id
        = this->frame_name_;
  this->sonar_image_msg_.header.stamp.sec
        = this->sensor_update_time_.sec;
  this->sonar_image_msg_.header.stamp.nsec
        = this->sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->sonar_image_msg_.header,
                                  sensor_msgs::image_encodings::TYPE_32FC1,
                                  Intensity_image);
  // from cv_bridge to sensor_msgs::Image
  img_bridge.toImageMsg(this->sonar_image_msg_);

  this->sonar_image_pub_.publish(this->sonar_image_msg_);

  // ---------------------------------------- End of sonar calculation

  this->lock_.unlock();
}

/////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonarRay::UpdatePointCloud(const sensor_msgs::PointCloud2ConstPtr& _msg)
{
  this->lock_.lock();

  // ROS_INFO_STREAM("================== pt  ========1===");
  this->point_cloud_image_.create(this->height, this->width, CV_32FC1);

  // sensor_msgs::PointCloud2Modifier pcd_modifier(const_cast<sensor_msgs::PointCloud2*>(_msg));
  // pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  // pcd_modifier.resize(this->height * this->width);
  // ROS_INFO_STREAM("================== pt  ========2===");

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*_msg,*pcl_pointcloud);
  // ROS_INFO_STREAM("================== pt  ========3===");

  // sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  // sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  // sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  cv::MatIterator_<float> iter_image = this->point_cloud_image_.begin<float>();

  ROS_INFO_STREAM("================== pt  ========4===");
  // double hfov = this->parentSensor->HorzFOV();
  // double hfl = static_cast<double>(this->width) / (2.0 * tan(hfov/2.0));
  // double vfov = this->parentSensor->VertFOV();
  // double vfl = static_cast<double>(this->height) / (2.0 * tan(vfov/2.0));

  // ROS_INFO_STREAM(pcl_pointcloud->points.size());
  // ROS_INFO_STREAM(pcl_pointcloud->points[1].x);
  // ROS_INFO_STREAM(this->height);
  // ROS_INFO_STREAM(this->width);
  // ROS_INFO_STREAM(this->width*this->height);
  // ROS_INFO_STREAM(pcl_pointcloud->points[this->width*this->height - 1].x);



  int index = 0;
  for (uint32_t j = 0; j < this->height; j++)
  {
    // for (uint32_t i = 0; i < this->width;
    //      i++, ++iter_x, ++iter_y, ++iter_z, ++iter_image)
    for (uint32_t i = 0; i < this->width; i++, ++iter_image, ++index)
    {


        pcl::PointXYZI point = pcl_pointcloud->at(j, this->width - i - 1);
        // Eigen::Vector3i xyz = point.getXYZVector3i();

                // result.at<cv::Vec3b>(h,w)[0] = rgb[2];
                // result.at<cv::Vec3b>(h,w)[1] = rgb[1];
                // result.at<cv::Vec3b>(h,w)[2] = rgb[0];
        this->point_cloud_image_.at<float>(j, i)
          = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

      // if (depth > this->point_cloud_cutoff_)
      // {
        // *iter_image = sqrt(*iter_x * *iter_x +
        //                    *iter_y * *iter_y +
        //                    *iter_z * *iter_z);
        // *iter_image = sqrt(pcl_pointcloud->points[index].x * pcl_pointcloud->points[index].x +
        //                   pcl_pointcloud->points[index].y * pcl_pointcloud->points[index].y +
        //                   pcl_pointcloud->points[index].z * pcl_pointcloud->points[index].z);

        if (isnan(*iter_image))
          *iter_image = 100000.0;
      // }
      // else  // point in the unseeable range
      // {
      //   *iter_image = 0.0;
      // }
    }
  }

  ROS_INFO_STREAM("================== pt  ========5===");
  this->lock_.unlock();
}

/////////////////////////////////////////////////
// Precalculation of corrector sonar calculation
void NpsGazeboRosMultibeamSonarRay::ComputeCorrector()
{
  double hFOV = this->parentSensor->HorzFOV();
  double hPixelSize = hFOV / this->width;
  double fl = static_cast<double>(width) / (2.0 * tan(hFOV/2.0));
  // Beam culling correction precalculation
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    float beam_azimuthAngle = atan2(static_cast<double>(beam) -
                        0.5 * static_cast<double>(width-1), fl);
    for (size_t beam_other = 0; beam_other < nBeams; beam_other ++)
    {
      float beam_azimuthAngle_other = atan2(static_cast<double>(beam_other) -
                        0.5 * static_cast<double>(width-1), fl);
      float azimuthBeamPattern =
        unnormalized_sinc(M_PI * 0.884 / hPixelSize
        * sin(beam_azimuthAngle-beam_azimuthAngle_other));
      this->beamCorrector[beam][beam_other] = abs(azimuthBeamPattern);
      this->beamCorrectorSum += pow(azimuthBeamPattern, 2);
    }
  }
  this->beamCorrectorSum = sqrt(this->beamCorrectorSum);
}

/////////////////////////////////////////////////
cv::Mat NpsGazeboRosMultibeamSonarRay::ComputeNormalImage(cv::Mat& depth)
{
  // filters
  cv::Mat_<float> f1 = (cv::Mat_<float>(3, 3) << 1,  2,  1,
                                                 0,  0,  0,
                                                -1, -2, -1) / 8;

  cv::Mat_<float> f2 = (cv::Mat_<float>(3, 3) << 1, 0, -1,
                                                 2, 0, -2,
                                                 1, 0, -1) / 8;

  cv::Mat f1m, f2m;
  cv::flip(f1, f1m, 0);
  cv::flip(f2, f2m, 1);

  cv::Mat n1, n2;
  cv::filter2D(depth, n1, -1, f1m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(depth, n2, -1, f2m, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::Mat no_readings;
  cv::erode(depth == 0, no_readings, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  // cv::dilate(no_readings, no_readings, cv::Mat(),
  //            cv::Point(-1, -1), 2, 1, 1);
  n1.setTo(0, no_readings);
  n2.setTo(0, no_readings);

  std::vector<cv::Mat> images(3);
  cv::Mat white = cv::Mat::ones(depth.rows, depth.cols, CV_32FC1);

  // NOTE: with different focal lengths, the expression becomes
  // (-dzx*fy, -dzy*fx, fx*fy)
  images.at(0) = n1;    // for green channel
  images.at(1) = n2;    // for red channel
  images.at(2) = 1.0/this->focal_length_*depth;  // for blue channel

  cv::Mat normal_image;
  cv::merge(images, normal_image);

  for (int i = 0; i < normal_image.rows; ++i)
  {
    for (int j = 0; j < normal_image.cols; ++j)
    {
      cv::Vec3f& n = normal_image.at<cv::Vec3f>(i, j);
      n = cv::normalize(n);
      float& d = depth.at<float>(i, j);
    }
  }
  return normal_image;
}

/////////////////////////////////////////////////
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