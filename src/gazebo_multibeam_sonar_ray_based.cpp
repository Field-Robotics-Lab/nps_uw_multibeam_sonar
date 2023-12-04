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
#include <pcl/features/normal_3d.h>

#include <nps_uw_multibeam_sonar/sonar_calculation_cuda.cuh>

#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <functional>
#include <nps_uw_multibeam_sonar/gazebo_multibeam_sonar_ray_based.hh>
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
  this->point_cloud_connect_count_ = 0;
  this->sonar_image_connect_count_ = 0;

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

  this->width = this->parentSensor->RangeCount();
  this->height = this->parentSensor->VerticalRangeCount();
  // this->format = this->laserCamera->ImageFormat();
  this->format = "R8G8B8";
  this->newLaserFrameConnection = this->laserCamera->ConnectNewLaserFrame(
      std::bind(&NpsGazeboRosMultibeamSonarRay::OnNewLaserFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);

  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
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

  this->constMu = true;
  this->mu = 1e-3;  // default constant mu

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
  this->elevation_angles = new float[this->nRays];

  // Print sonar calculation settings
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("============   SONAR PLUGIN LOADED   =============");
  ROS_INFO_STREAM("==================================================");
  ROS_INFO_STREAM("============       RAY VERSION       =============");
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
      ROS_INFO_STREAM("Raw data at " << "/tmp/SonarRawData_{numbers}.csv"
                      << " every " << this->writeInterval << " frames");
      ROS_INFO_STREAM("Beam angles at /tmp/SonarRawData_beam_angles.csv");
      ROS_INFO_STREAM("");

      struct stat buffer;
      std::string logfilename("/tmp/SonarRawData_000001.csv");
      std::string logfilename_angles("/tmp/SonarRawData_beam_angles.csv");
      if (stat (logfilename.c_str(), &buffer) == 0)
        system("rm /tmp/SonarRawData*.csv");
      if (stat (logfilename_angles.c_str(), &buffer) == 0)
        system("rm /tmp/SonarRawData_beam_angles.csv");
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
  rng.fill(this->rand_image, cv::RNG::NORMAL, 0.0f, 1.0f);

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
  // Publisher for point cloud
  ros::SubscribeOptions so =
  ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
      "/" + this->point_cloud_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonarRay::UpdatePointCloud, this, _1),
      ros::VoidPtr(), &this->pointCloudSubQueue);
  this->VelodyneGpuLaserPointCloud = this->rosnode_->subscribe(so);
  // Spin up the queue helper thread.
  this->pointCloudSubQueueThread = std::thread(std::bind(
      &NpsGazeboRosMultibeamSonarRay::pointCloudSubThread, this));

  ros::AdvertiseOptions point_cloud_ao =
    ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
      this->point_cloud_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonarRay::PointCloudConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonarRay::PointCloudDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);

  // Publisher for normal image
  ros::AdvertiseOptions normal_image_ao =
    ros::AdvertiseOptions::create<sensor_msgs::Image>(
      "/" + this->point_cloud_topic_name_ + "_normal_image", 1,
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->normal_image_pub_ = this->rosnode_->advertise(normal_image_ao);

  // Publisher for sonar raw data
  ros::AdvertiseOptions sonar_image_raw_ao =
    ros::AdvertiseOptions::create<marine_acoustic_msgs::ProjectedSonarImage>(
      this->sonar_image_raw_topic_name_, 1,
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageConnect, this),
      boost::bind(&NpsGazeboRosMultibeamSonarRay::SonarImageDisconnect, this),
      ros::VoidPtr(), &this->camera_queue_);
  this->sonar_image_raw_pub_ = this->rosnode_->advertise(sonar_image_raw_ao);

  // Publisher for sonar image
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

/////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonarRay::OnNewLaserFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string &_format)
{
  this->sensor_update_time_ = this->parentSensor->LastMeasurementTime();
  if (this->parentSensor->IsActive())
  {
    if (this->sonar_image_connect_count_ > 0
        && this->point_cloud_image_.size().width != 0 )
      this->ComputeSonarImage();
  }
  else
  {
    if (this->sonar_image_connect_count_ > 0)
      this->parentSensor->SetActive(true);
  }
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
  double vPixelSize = vFOV / (this->height-1);
  double hPixelSize = hFOV / (this->width-1);

  if (this->beamCorrectorSum == 0)
    ComputeCorrector();

  // Default value for reflectivity
  if (this->reflectivityImage.rows == 0)
    this->reflectivityImage = cv::Mat(width, height, CV_32FC1, cv::Scalar(this->mu));

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
                  this->elevation_angles, // _ray_elevationAngles
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

      // write beam (azimuth) angles
      if (this->writeNumber == 1)
      {
        std::stringstream filename_angle;
        filename_angle << "/tmp/SonarRawData_beam_angles.csv";
        writeLog.open(filename_angle.str().c_str(), std::ios_base::app);
        filename_angle.clear();
        writeLog << "# Raw Sonar Data Log \n";
        writeLog << "# Beam (azimuth) angles of rays\n";
        writeLog << "#  nBeams : " << nBeams << "\n";
        writeLog << "# Simulation time : " << time << "\n";
        for (size_t i = 0; i < this->azimuth_angles.size(); i++)
          writeLog << this->azimuth_angles[i]<< "\n";
        writeLog.close();
      }

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
  marine_acoustic_msgs::PingInfo ping_info_msg_;
  ping_info_msg_.frequency = this->sonarFreq;
  ping_info_msg_.sound_speed = this->soundSpeed;
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    ping_info_msg_.rx_beamwidths.push_back(static_cast<float>(hFOV/floor(nBeams*2.0-2.0)*2.0));
    ping_info_msg_.tx_beamwidths.push_back(static_cast<float>(vFOV));
  }
  this->sonar_image_raw_msg_.ping_info = ping_info_msg_;
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    geometry_msgs::Vector3 beam_direction;
    beam_direction.x = cos(azimuth_angles[beam]);
    beam_direction.y = sin(azimuth_angles[beam]);
    beam_direction.z = 0.0;
    this->sonar_image_raw_msg_.beam_directions.push_back(beam_direction);
  }
  std::vector<float> ranges;
  for (size_t i = 0; i < P_Beams[0].size(); i ++)
    ranges.push_back(rangeVector[i]);
  this->sonar_image_raw_msg_.ranges = ranges;
  marine_acoustic_msgs::SonarImageData sonar_image_data;
  sonar_image_data.is_bigendian = false;
  sonar_image_data.dtype = 0; //DTYPE_UINT8
  sonar_image_data.beam_count = nBeams;
  //this->sonar_image_raw_msg_.data_size = 1;  // sizeof(float) * nFreq * nBeams;
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
  sonar_image_data.data = intensities;
  this->sonar_image_raw_msg_.image = sonar_image_data;
  this->sonar_image_raw_pub_.publish(this->sonar_image_raw_msg_);



  // Construct visual sonar image for rqt plot in sensor::image msg format
  cv_bridge::CvImage img_bridge;

  // Generate image of CV_8UC1
  cv::Mat Intensity_image = cv::Mat::zeros(cv::Size(nBeams, nFreq), CV_8UC1);

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
    const float center = this->azimuth_angles[b];
    float begin = 0.0, end = 0.0;
    if (b == 0)
    {
      end = (this->azimuth_angles[b + 1] + center) / 2.0;
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
      end = (this->azimuth_angles[b + 1] + center) / 2.0;
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
      const int intensity = floor(10.0*log(abs(P_Beams[nBeams - 1 - b][r])));
      const float begin = angles[b].begin + ThetaShift,
                  end = angles[b].end + ThetaShift;
      const float rad = static_cast<float>(radius) * range/rangeMax;
      // Assume angles are in image frame x-right, y-down
      cv::ellipse(Intensity_image, origin, cv::Size(rad, rad), 0.0,
                  begin * 180.0/M_PI, end * 180.0/M_PI,
                  intensity, binThickness);
    }
  }

  // Normlize and colorize
  cv::normalize(Intensity_image,Intensity_image,
                -255 + this->plotScaler/10*255, 255, cv::NORM_MINMAX);
  cv::Mat Itensity_image_color;
  cv::applyColorMap(Intensity_image, Itensity_image_color, cv::COLORMAP_HOT);

  // Publish final sonar image
  this->sonar_image_msg_.header.frame_id
        = this->frame_name_;
  this->sonar_image_msg_.header.stamp.sec
        = this->sensor_update_time_.sec;
  this->sonar_image_msg_.header.stamp.nsec
        = this->sensor_update_time_.nsec;
  img_bridge = cv_bridge::CvImage(this->sonar_image_msg_.header,
                                  sensor_msgs::image_encodings::BGR8,
                                  Itensity_image_color);
  // from cv_bridge to sensor_msgs::Image
  img_bridge.toImageMsg(this->sonar_image_msg_);

  this->sonar_image_pub_.publish(this->sonar_image_msg_);

  // ---------------------------------------- End of sonar calculation

  // Still publishing the normal image (just because)
  this->normal_image_msg_.header.frame_id
        = this->frame_name_;
  this->normal_image_msg_.header.stamp.sec
        = this->sensor_update_time_.sec;
  this->normal_image_msg_.header.stamp.nsec
        = this->sensor_update_time_.nsec;
  cv::Mat normal_image8;
  normal_image.convertTo(normal_image8, CV_8UC3, 255.0);
  img_bridge = cv_bridge::CvImage(this->normal_image_msg_.header,
                                  sensor_msgs::image_encodings::RGB8,
                                  normal_image8);
  img_bridge.toImageMsg(this->normal_image_msg_);
  // from cv_bridge to sensor_msgs::Image
  this->normal_image_pub_.publish(this->normal_image_msg_);

  this->lock_.unlock();
}

/////////////////////////////////////////////////
void NpsGazeboRosMultibeamSonarRay::UpdatePointCloud(const sensor_msgs::PointCloud2ConstPtr& _msg)
{
  this->lock_.lock();

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*_msg,*pcl_pointcloud);

  this->point_cloud_image_.create(this->height, this->width, CV_32FC1);
  cv::MatIterator_<float> iter_image = this->point_cloud_image_.begin<float>();
  double hFOV = this->parentSensor->HorzFOV();

  // calculate azimuth/elevation angles
  bool angles_calculation_flag = false;
  if (this->azimuth_angles.size() == 0)
    angles_calculation_flag = true;

  for (int j = 0; j < this->nRays; j++)
  {
    if (angles_calculation_flag)
    {
      const double Diff = this->parentSensor->VerticalAngleMax().Radian()
                          - this->parentSensor->VerticalAngleMin().Radian();
      this->elevation_angles[j] = ( j * Diff / (this->nRays - 1 )
                          + this->parentSensor->VerticalAngleMin().Radian() );
    }

    for (int i = 0; i < this->nBeams; i++, ++iter_image)
    {
      pcl::PointXYZI point = pcl_pointcloud->at(j, this->width - i - 1);

      this->point_cloud_image_.at<float>(j, i)
        = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (angles_calculation_flag && j == 0)
      {
        const double Diff = this->parentSensor->AngleMax().Radian()
                            - this->parentSensor->AngleMin().Radian();
        this->azimuth_angles.push_back( i * Diff / (this->nBeams - 1 )
                            + this->parentSensor->AngleMin().Radian() );
      }

      if (isnan(*iter_image))
        *iter_image = 100000.0;
    }
  }

    if (this->point_cloud_connect_count_ > 0)
      this->point_cloud_pub_.publish(this->point_cloud_msg_);

  this->lock_.unlock();
}

/////////////////////////////////////////////////
// Precalculation of corrector sonar calculation
void NpsGazeboRosMultibeamSonarRay::ComputeCorrector()
{
  double hFOV = this->parentSensor->HorzFOV();
  double hPixelSize = hFOV / (this->width-1);
  // Beam culling correction precalculation
  for (size_t beam = 0; beam < nBeams; beam ++)
  {
    for (size_t beam_other = 0; beam_other < nBeams; beam_other ++)
    {
      float azimuthBeamPattern =
        unnormalized_sinc(M_PI * 0.884 / hPixelSize
        * sin(this->azimuth_angles[beam]-this->azimuth_angles[beam_other]));
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

}