/// @file mrcal_image_processing.hpp
/// @brief Provides a configurable interface for the processing needed to
/// convert mrcal models to ROS supported ones

#ifndef INC_GUARD_MRCAL_IMAGE_PROCESSING_HPP
#define INC_GUARD_MRCAL_IMAGE_PROCESSING_HPP

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <mrcal/mrcal-types.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

extern "C" {
#include <mrcal/mrcal.h>
}

namespace mrcal_ros
{

struct ImgSensorData
{
  sensor_msgs::msg::CameraInfo info;
  std::shared_ptr<sensor_msgs::msg::Image const> image;
};

class IMrCalProcessor
{
public:
  virtual ImgSensorData
  process(sensor_msgs::msg::Image::ConstSharedPtr img) = 0;
};

} // namespace mrcal_ros

#endif
