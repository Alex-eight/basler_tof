/*
 * Copyright (C) 2017, DFKI GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author:
 *         Martin Günther <martin.guenther@dfki.de>
 */

#include <ConsumerImplHelper/ToFCamera.h>
#include <GenTL/PFNC.h>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dynamic_reconfigure/server.h>
#include <basler_tof/BaslerToFConfig.h>

using namespace GenTLConsumerImplHelper;
using namespace GenApi;
using namespace std;

ros::Publisher cloud_pub_;
ros::Publisher intensity_pub_;
ros::Publisher intensity_ci_pub_;
ros::Publisher confidence_pub_;
ros::Publisher confidence_ci_pub_;
ros::Publisher depth_pub_;
ros::Publisher depth_ci_pub_;
std::string frame_id_;
std::string device_id_;
std::string camera_name_;
CToFCamera camera_;

boost::shared_ptr<camera_info_manager::CameraInfoManager> intensity_info_manager_;
boost::shared_ptr<camera_info_manager::CameraInfoManager> confidence_info_manager_;
boost::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_;


bool publish(const BufferParts& parts, ros::Time acquisition_time)
{
  if (parts.size() != 3)
  {
    ROS_ERROR("Expected 3 parts, got %zu!", parts.size());
    return false;
  }

  // If the point cloud is enabled, the first part always contains the point cloud data.
  if (parts[0].dataFormat != PFNC_Coord3D_ABC32f)
  {
    ROS_ERROR("Unexpected data format for the first image part. Coord3D_ABC32f is expected.");
    return false;
  }

  if (parts[1].dataFormat != PFNC_Mono16)
  {
    ROS_ERROR("Unexpected data format for the second image part. Mono16 is expected.");
    return false;
  }

  // ----- publish point cloud
  const size_t nPixel = parts[0].width * parts[0].height;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  cloud->header.frame_id = frame_id_;
  cloud->header.stamp = pcl_conversions::toPCL(acquisition_time);
  cloud->width = parts[0].width;
  cloud->height = parts[0].height;
  cloud->is_dense = false;
  cloud->points.resize(nPixel);

  CToFCamera::Coord3D *pPoint = static_cast<CToFCamera::Coord3D*>(parts[0].pData);
  uint16_t *pIntensity = static_cast<uint16_t*>(parts[1].pData);

  for (size_t i = 0; i < nPixel; ++i)
  {
    pcl::PointXYZI &p = cloud->points[i];
    if (pPoint->IsValid())
    {
      p.x = 0.001f * pPoint->x;
      p.y = 0.001f * pPoint->y;
      p.z = 0.001f * pPoint->z;
      p.intensity = *pIntensity;
    }
    else
    {
      p.x = std::numeric_limits<float>::quiet_NaN();
      p.y = std::numeric_limits<float>::quiet_NaN();
      p.z = std::numeric_limits<float>::quiet_NaN();
    }
    pPoint++;
    pIntensity++;
  }

  cloud_pub_.publish(cloud);

  // ----- publish intensity image
  cv_bridge::CvImage intensity_cvimg;
  intensity_cvimg.encoding = sensor_msgs::image_encodings::MONO16;
  intensity_cvimg.header.frame_id = frame_id_;
  intensity_cvimg.header.stamp = acquisition_time;
  intensity_cvimg.image = cv::Mat(parts[1].height, parts[1].width, CV_16UC1, parts[1].pData).clone();

  // uncomment these two lines for cameracalibrator.py
  // intensity_cvimg.image.convertTo(intensity_cvimg.image, CV_8U, 1.0 / 256.0);
  // intensity_cvimg.encoding = sensor_msgs::image_encodings::MONO8;

  intensity_pub_.publish(intensity_cvimg.toImageMsg());

  sensor_msgs::CameraInfoPtr intensity_info_msg(new sensor_msgs::CameraInfo(intensity_info_manager_->getCameraInfo()));
  intensity_info_msg->header.stamp    = acquisition_time;
  intensity_info_msg->header.frame_id = frame_id_;
  intensity_ci_pub_.publish(intensity_info_msg);

  // ----- publish confidence image
  cv_bridge::CvImage confidence_cvimg;
  confidence_cvimg.encoding = sensor_msgs::image_encodings::MONO16;
  confidence_cvimg.header.frame_id = frame_id_;
  confidence_cvimg.header.stamp = acquisition_time;
  confidence_cvimg.image = cv::Mat(parts[2].height, parts[2].width, CV_16UC1, parts[2].pData).clone();
  confidence_pub_.publish(confidence_cvimg.toImageMsg());

  sensor_msgs::CameraInfoPtr confidence_info_msg(new sensor_msgs::CameraInfo(confidence_info_manager_->getCameraInfo()));
  confidence_info_msg->header.stamp    = acquisition_time;
  confidence_info_msg->header.frame_id = frame_id_;
  confidence_ci_pub_.publish(confidence_info_msg);

  // ----- publish depth image
  pPoint = static_cast<CToFCamera::Coord3D*>(parts[0].pData);

  cv_bridge::CvImage depth_cvimg;
  depth_cvimg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_cvimg.header.frame_id = frame_id_;
  depth_cvimg.header.stamp = acquisition_time;
  depth_cvimg.image = cv::Mat(parts[0].height, parts[0].width, CV_16UC1);
  depth_cvimg.image.setTo(0);

  for (size_t i = 0; i < parts[0].height; i++)
  {
    for (size_t j = 0; j < parts[0].width; j++)
    {
      depth_cvimg.image.at<uint16_t>(i, j) = static_cast<uint16_t>(pPoint->z);   // can be NaN; should be in mm
      pPoint++;
    }
  }

  depth_pub_.publish(depth_cvimg.toImageMsg());

  sensor_msgs::CameraInfoPtr depth_info_msg(new sensor_msgs::CameraInfo(depth_info_manager_->getCameraInfo()));
  depth_info_msg->header.stamp    = acquisition_time;
  depth_info_msg->header.frame_id = frame_id_;
  depth_ci_pub_.publish(depth_info_msg);

  return true;
}

template <typename T>
void round_to_increment(T &param, T min, T inc)
{
  // Allowed value must follow value ==  min + n * inc, n integer.
  param = min + ( (int) (( param - min ) / inc )) * inc;
}

template <class T>
bool get_and_check_parameter(const char* name, const char* type, T& ptrParameter)
{
  CValuePtr ptrGenericParameter = camera_.GetParameter(name);
  if ( ! ptrGenericParameter.IsValid() )
  {
    ROS_WARN_STREAM("Device doesn't support a " << "name" << " parameter.");
    return false;
  }

  ptrParameter = ptrGenericParameter; // casts CValuePtr to T
  if ( ! ptrParameter.IsValid() )
  {
    // parameter is supported, but wrong type
    ROS_WARN_STREAM("Parameter " << name << "is not of type " << type);
    return false;
  }

  if ( ! GenApi::IsAvailable(ptrGenericParameter ) )
  {
    ROS_WARN_STREAM("The current state of the device doesn't allow to access the " << name << " parameter.");
    return false;
  }
  if ( ! GenApi::IsWritable(ptrGenericParameter) )
  {
    ROS_WARN_STREAM("The " << name << " parameter is read-only.");
    return false;
  }
  return true;
}

template<class T, class Ptr>
void set_value( const char* name, const char* type, T& value)
{
  try
  {
    Ptr ptrParameter;
    if ( ! get_and_check_parameter(name, type, ptrParameter) )
    {
      // parameter not supported by the device or not writable
      return;
    }

    // Clip value to boundaries and adjust it according the parameters increments.
    const T min = static_cast<T>(ptrParameter->GetMin());
    const T max = static_cast<T>(ptrParameter->GetMax());
    if ( value < min )
    {
      ROS_INFO_STREAM("Desired value of " << value << " is smaller than the current minimum value of the "
                       << name << " parameter (" << min << "). Value will be clipped." );
      value = min;
    }
    else if ( value > max )
    {
      ROS_INFO_STREAM("Desired value of " << value << " is greater than the current maximum value of the "
                       << name << " parameter (" << max << "). Value will be clipped." );
      value = max;
    }
    const T inc = static_cast<int>(ptrParameter->GetInc());
    if ( inc != 1 )
    {
      round_to_increment(value, min, inc);
    }
    ptrParameter->SetValue(value);
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Exception occurred in set_value: " << e.GetDescription());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception occurred in set_value: " << e.what());
  }
  catch ( ... )
  {
    ROS_ERROR("Unknown exception occurred in set_value.");
  }
}

void set_value_int(const char* name, int& value)
{
  set_value<int, CIntegerPtr>(name, "integer", value);
}

void set_value_double( const char* name, double& value)
{
  set_value<double, CFloatPtr>(name, "double", value);
}

void set_value_boolean( const char* name, bool& value)
{
  try
  {
    CBooleanPtr ptrParameter;
    if ( ! get_and_check_parameter(name, "boolean", ptrParameter) )
    {
      // parameter not supported by the device or not writable
      return;
    }
    ptrParameter->SetValue(value);
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Exception occurred in set_value: " << e.GetDescription());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception occurred in set_value: " << e.what());
  }
  catch ( ... )
  {
    ROS_ERROR("Unknown exception occurred in set_value.");
  }
}

void set_value_enum( const char* name, std::string& value)
{
  try
  {
    CValuePtr ptrParameter;
    if ( ! get_and_check_parameter(name, "enum", ptrParameter) )
    {
      // parameter not supported by the device or not writable
      return;
    }
    ptrParameter->FromString(value.c_str());
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Exception occurred in set_value: " << e.GetDescription());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception occurred in set_value: " << e.what());
  }
  catch ( ... )
  {
    ROS_ERROR("Unknown exception occurred in set_value.");
  }
}

void update_config(basler_tof::BaslerToFConfig &new_config, uint32_t level)
{

  try
  {
    // Configuration of Processing Mode must be done before configuring the exposure time(s) and acquisition rate.
    set_value_enum("ProcessingMode", new_config.processing_mode);

    set_value_boolean("Binning", new_config.binning);
    set_value_int("DeviceChannel", new_config.device_channel);
    set_value_enum("Rectification", new_config.rectification);
    set_value_int("DeviceCalibOffset", new_config.calibration_range_offset);
    set_value_int("DepthMin", new_config.minimum_depth);
    set_value_int("DepthMax", new_config.maximum_depth);

    double val = new_config.frame_rate;
    set_value_double("AcquisitionFrameRate", val);
    new_config.frame_rate = val;

    // Auto exposure can be only activated in standard processing mode.
    // ExposureTime is only valid when exposure_auto is "false".
    if (new_config.exposure_auto && new_config.processing_mode.compare("Hdr") != 0)
    {
      std::string val = "Continuous";
      set_value_enum("ExposureAuto", val);

      // Agility and Delay are only valid when exposure_auto is "true"
      set_value_double("Agility",new_config.exposure_agility);
      set_value_int("Delay", new_config.exposure_delay);
    }
    else
    {
      new_config.exposure_auto = false;
      std::string str_val = "Off";
      set_value_enum("ExposureAuto", str_val);

      // Set first exposure time
      CIntegerPtr ptrExposureTimeSelector(camera_.GetParameter("ExposureTimeSelector"));
      if (ptrExposureTimeSelector.IsValid() && GenApi::IsWritable(ptrExposureTimeSelector) )
      {
        ptrExposureTimeSelector->SetValue(0);
      }

      double d_val = new_config.exposure_time_0;
      set_value_double("ExposureTime", d_val);
      new_config.exposure_time_0 = d_val;

      // The second exposure time is only valid when in HDR mode
      if ( new_config.processing_mode.compare("Hdr") == 0 && ptrExposureTimeSelector.IsValid() && GenApi::IsWritable(ptrExposureTimeSelector) )
      {
        // select 2nd exposure time
        ptrExposureTimeSelector->SetValue(1);
        // set the 2nd exposure time
        d_val = new_config.exposure_time_1;
        set_value_double("ExposureTime", d_val);
        new_config.exposure_time_1 = d_val;
      }
    }

    set_value_int("ConfidenceThreshold", new_config.confidence_threshold);
    set_value_boolean("FilterSpatial", new_config.spatial_filter);
    set_value_boolean("FilterTemporal", new_config.temporal_filter);
    set_value_int("FilterStrength", new_config.temporal_strength);

    // the range filter must not be enabled when the spatial filter is disabled
    if ( new_config.spatial_filter)
    {
      set_value_boolean("FilterRange", new_config.range_filter);
    }
    set_value_int("OutlierTolerance", new_config.outlier_tolerance);
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Exception occurred in update_config: " << endl << e.GetDescription());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception occurred in update_config: " << endl << e.what());
  }
  catch ( ... )
  {
    ROS_ERROR("Unknown exception occurred in update_config.");
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "basler_tof_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::NodeHandle intensity_nh("intensity");
  ros::NodeHandle confidence_nh("confidence");
  ros::NodeHandle depth_nh("depth");

  pn.param("frame_id", frame_id_, std::string("camera_optical_frame"));
  if (!pn.getParam("device_id", device_id_))
  {
    ROS_WARN("~device_id is not set! Using first device.");
    device_id_ = "#1";
  }

  cloud_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 10);
  intensity_pub_ = n.advertise<sensor_msgs::Image>("intensity/image_raw", 10);
  confidence_pub_ = n.advertise<sensor_msgs::Image>("confidence/image_raw", 10);
  depth_pub_ = n.advertise<sensor_msgs::Image>("depth/image_raw", 10);
  intensity_ci_pub_ = n.advertise<sensor_msgs::CameraInfo>("intensity/camera_info", 10);
  confidence_ci_pub_ = n.advertise<sensor_msgs::CameraInfo>("confidence/camera_info", 10);
  depth_ci_pub_ = n.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 10);

  intensity_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(intensity_nh);
  confidence_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(confidence_nh);
  depth_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(depth_nh);

  int exitCode = EXIT_FAILURE;

  try
  {
    CToFCamera::InitProducer();

    if (device_id_ == "#1")
    {
      ROS_INFO("Opening first camera.");
      camera_.OpenFirstCamera();
    }
    else
    {
      CameraList cameraList = camera_.EnumerateCameras();
      bool found = false;
      for (CameraList::const_iterator it = cameraList.begin(); it != cameraList.end(); ++it)
      {
        ROS_INFO("Found camera device with serial: %s", it->strSerialNumber.c_str());
        if (it->strSerialNumber == device_id_)
        {
          ROS_INFO("Serial matches device_id, opening camera.");
          found = true;
          camera_.Open(*it);
          break;
        }
      }
      if (!found)
      {
        ROS_FATAL("No camera with device_id '%s' found, exiting!", device_id_.c_str());
        return EXIT_FAILURE;
      }
    }

    std::string camera_name_ = camera_.GetCameraInfo().strModelName
                               + "_" + camera_.GetCameraInfo().strSerialNumber;
    if (!intensity_info_manager_->setCameraName(camera_name_) ||
        !confidence_info_manager_->setCameraName(camera_name_) ||
        !depth_info_manager_->setCameraName(camera_name_))
    {
      ROS_WARN_STREAM("[" << camera_name_
                      << "] name not valid"
                      << " for camera_info_manager");
    }

    std::string camera_info_url;
    if (pn.getParam("camera_info_url", camera_info_url))
    {
      if (!intensity_info_manager_->validateURL(camera_info_url) ||
          !confidence_info_manager_->validateURL(camera_info_url) ||
          !depth_info_manager_->validateURL(camera_info_url))
      {
        ROS_WARN("camera_info_url invalid: %s", camera_info_url.c_str());
      }
      else
      {
        intensity_info_manager_->loadCameraInfo(camera_info_url);
        confidence_info_manager_->loadCameraInfo(camera_info_url);
        depth_info_manager_->loadCameraInfo(camera_info_url);
      }
    }

    ROS_INFO_STREAM("[" << camera_name_ << "] opened.");

    ROS_INFO_STREAM("DeviceVendorName:      " << CStringPtr(camera_.GetParameter("DeviceVendorName"))->GetValue());
    ROS_INFO_STREAM("DeviceModelName:       " << CStringPtr(camera_.GetParameter("DeviceModelName"))->GetValue());
    ROS_INFO_STREAM("DeviceVersion:         " << CStringPtr(camera_.GetParameter("DeviceVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceFirmwareVersion: " << CStringPtr(camera_.GetParameter("DeviceFirmwareVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceDriverVersion:   " << CStringPtr(camera_.GetParameter("DeviceDriverVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceSerialNumber:    " << CStringPtr(camera_.GetParameter("DeviceSerialNumber"))->GetValue());

    ROS_INFO_STREAM("DeviceCalibVersion:    " << CIntegerPtr(camera_.GetParameter("DeviceCalibVersion"))->GetValue());
    ROS_INFO_STREAM("DeviceCalibState:      " << CEnumerationPtr(camera_.GetParameter("DeviceCalibState"))->ToString());
    ROS_INFO_STREAM("DeviceCalibOffset:     " << CIntegerPtr(camera_.GetParameter("DeviceCalibOffset"))->GetValue());

    ROS_INFO_STREAM("DeviceTemperature:     " << CFloatPtr(camera_.GetParameter("DeviceTemperature"))->GetValue() << " degrees C");


    // Parameterize the camera to send 3D coordinates and intensity data
    CEnumerationPtr ptrImageComponentSelector = camera_.GetParameter("ImageComponentSelector");
    CBooleanPtr ptrImageComponentEnable = camera_.GetParameter("ImageComponentEnable");
    CEnumerationPtr ptrPixelFormat = camera_.GetParameter("PixelFormat");

    ptrImageComponentSelector->FromString("Range");
    ptrImageComponentEnable->SetValue(true);
    ptrPixelFormat->FromString("Coord3D_ABC32f");

    ptrImageComponentSelector->FromString("Intensity");
    ptrImageComponentEnable->SetValue(true);
    ptrPixelFormat->FromString("Mono16");

    ptrImageComponentSelector->FromString("Confidence");
    ptrImageComponentEnable->SetValue(true);
    ptrPixelFormat->FromString("Confidence16");

    dynamic_reconfigure::Server<basler_tof::BaslerToFConfig> dynamic_reconfigure_server;
    dynamic_reconfigure::Server<basler_tof::BaslerToFConfig>::CallbackType f;
    f = boost::bind(&update_config, _1, _2);
    dynamic_reconfigure_server.setCallback(f);

    const size_t nBuffers = 3;  // Number of buffers to be used for grabbing.

    // Allocate the memory buffers and prepare image acquisition.
    camera_.PrepareAcquisition( nBuffers );

    // Enqueue all buffers to be filled with image data.
    for ( size_t i = 0; i < nBuffers; ++i )
    {
        camera_.QueueBuffer( i );
    }

    // Start the acquisition engine.
    camera_.StartAcquisition();
    camera_.IssueAcquisitionStartCommand(); // The camera continuously sends data now.


    while (ros::ok())
    {
      ros::spinOnce();

      GrabResult grabResult;

      // Get next acquired frame frome the driver's output queue.
      camera_.GetGrabResult( grabResult, 1000 );

      // Save 3D data
      if (grabResult.status == GrabResult::Ok)
      {
        BufferParts parts;
        camera_.GetBufferParts( grabResult, parts);
        publish(parts, ros::Time::now());
      }
      else
      {
        ROS_ERROR("Failed to grab an image.");
      }

      // Requeue the buffer to the buffer's input queue.
      if (grabResult.status != GrabResult::Timeout)
      {
        camera_.QueueBuffer( grabResult.hBuffer );
      }
    }

    camera_.Close();
    exitCode = EXIT_SUCCESS;
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Exception occurred: " << endl << e.GetDescription());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception occurred: " << endl << e.what());
  }
  catch ( ... )
  {
    ROS_ERROR("Unknown exception occurred.");
  }

  if (CToFCamera::IsProducerInitialized())
    CToFCamera::TerminateProducer();  // Won't throw any exceptions

  return exitCode;
}

