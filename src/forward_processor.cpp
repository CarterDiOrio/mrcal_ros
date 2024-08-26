#include "mrcal_ros/foward_processor.hpp"
#include "mrcal_ros/mrcal_image_processing.hpp"

namespace mrcal_ros
{

ForwardProcessor::ForwardProcessor(std::unique_ptr<mrcal_cameramodel_t> model)
: model{std::move(model)} {}

ImgSensorData ForwardProcessor::process(sensor_msgs::msg::Image::ConstSharedPtr img)
{

  sensor_msgs::msg::CameraInfo info;
  info.header = img->header;
  info.height = model->imagersize[1];
  info.width = model->imagersize[0];

  // set intrinsics
  std::fill(std::begin(info.k), std::end(info.k), 0.0);
  info.k[0] = model->intrinsics[0];
  info.k[2] = model->intrinsics[2];
  info.k[5] = model->intrinsics[1];
  info.k[6] = model->intrinsics[3];

  // set distortion
  size_t distortion_count = 0;
  switch (model->lensmodel.type) {
    case MRCAL_LENSMODEL_PINHOLE:
      distortion_count = 0;
      break;
    case MRCAL_LENSMODEL_OPENCV5:
      info.distortion_model = "plumb_bob";
      distortion_count = 5;
      break;
    case MRCAL_LENSMODEL_OPENCV8:
      info.distortion_model = "rational_polynomial";
      distortion_count = 8;
      break;
    default:
      throw std::logic_error("Forward Processor accepted incompatible lean lens model");
  }

  for (size_t idx = 0; idx < distortion_count; idx++) {
    info.d[idx] = model->intrinsics[4 + idx];
  }

  return {
    info,
    img
  };
}

}
