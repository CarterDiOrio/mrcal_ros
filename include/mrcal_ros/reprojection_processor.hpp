/// @file reprojection_processor.hpp
/// @brief Contains a IMrCalProcessor implementation that reprojects the image
/// to a different camera model

#ifndef INC_GUARD_REPROJECTION_PROCESSOR_HPP
#define INC_GUARD_REPROJECTION_PROCESSOR_HPP

#include "mrcal_ros/mrcal_image_processing.hpp"
#include <cv_bridge/cv_bridge.hpp>


namespace mrcal_ros
{

/// @brief Processor Implemenation that reprojects the image to a different camera model
class ReprojectionProcessor : public IMrCalProcessor
{
public:
  /// @brief Constructor, initializes the opencv reprojection maps
  /// @param from The camera model to reproject from
  /// @param to The camera model to reproject to
  ReprojectionProcessor(
    std::unique_ptr<mrcal_cameramodel_t> from,
    std::unique_ptr<mrcal_cameramodel_t> to);

  ImgSensorData
  process(sensor_msgs::msg::Image::ConstSharedPtr img) override final;

private:
  /// @brief The camera model to reproject from
  std::unique_ptr<mrcal_cameramodel_t const> from;

  /// @brief The camera model to reproject to
  std::unique_ptr<mrcal_cameramodel_t const> to;

  /// @brief The x reprojection map
  cv::Mat map_x;

  /// @brief the y reprojection map
  cv::Mat map_y;
};

}  // namespace mrcal_ros

#endif
