/// @file forward_process.hpp
/// @brief Contains a IMrCalProcessor implementation that forwards the image
/// without processing with the new intrinsics.

#ifndef INC_GUARD_FORWARD_PROCESSOR_HPP
#define INC_GUARD_FORWARD_PROCESSOR_HPP

#include "mrcal_ros/mrcal_image_processing.hpp"

namespace mrcal_ros
{

/// @brief Forwards the img with intrinsics, but no processing
class ForwardProcessor : public IMrCalProcessor
{
public:
  /// @brief Constructor
  /// @param model The model to publish camera info for
  ForwardProcessor(std::unique_ptr<mrcal_cameramodel_t> model);

  ImgSensorData
  process(sensor_msgs::msg::Image::ConstSharedPtr img) override final;

private:
  std::unique_ptr<mrcal_cameramodel_t> model;
};

} // namespace mrcal_ros

#endif
