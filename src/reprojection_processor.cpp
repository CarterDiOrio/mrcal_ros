#include "mrcal_ros/reprojection_processor.hpp"
#include "mrcal_ros/mrcal_image_processing.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <mrcal/mrcal-types.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace mrcal_ros
{

ReprojectionProcessor::ReprojectionProcessor(
  std::unique_ptr<mrcal_cameramodel_t> from,
  std::unique_ptr<mrcal_cameramodel_t> to)
:from{std::move(from)}, to{std::move(to)}
{
  // initialize the reprojection maps

  const auto to_width = to->imagersize[0];
  const auto to_height = to->imagersize[1];

  // create a grid of points over the destination image
  const auto points_to = [&to_width, &to_height] {
    std::vector<mrcal_point2_t> points;
    for (size_t r = 0; r < to_height; r++) {
      for (size_t c = 0; c < to_width; c++) {
        mrcal_point2_t point;
        point.x = c;
        point.y = r;
        points.push_back(point);
      }
    }
    return points;
  }();

  const auto rays = [&points_to, & lean_lens_model = *to] {
    std::vector<mrcal_point3_t> rays;
    rays.resize(points_to.size() + 10);
    mrcal_unproject(
      rays.data(),
      points_to.data(),
      points_to.size(),
      &lean_lens_model.lensmodel,
      lean_lens_model.intrinsics);
    return rays;
  }();

  const auto projected_points = [&rays, & rich_lens_model = *from] {
    std::vector<mrcal_point2_t> projected_points;
    projected_points.resize(rays.size() + 10);
    mrcal_project(
      projected_points.data(),
      NULL,   // do not need the gradients
      NULL,   // do not need the gradients
      rays.data(),
      rays.size(),
      &rich_lens_model.lensmodel,
      rich_lens_model.intrinsics);
    return projected_points;
  }();

  map_x = cv::Mat::zeros(
    {static_cast<int>(to_width), static_cast<int>(to_height)},
    CV_32FC1);
  map_y = cv::Mat::zeros(
    {static_cast<int>(to_width), static_cast<int>(to_height)},
    CV_32FC1);

  for (size_t r = 0; r < to_height; r++) {
    for (size_t c = 0; c < to_width; c++) {
      const auto idx = r * to_width + c;
      map_x.at<float>(r, c) = projected_points.at(idx).x;
      map_y.at<float>(r, c) = projected_points.at(idx).y;
    }
  }
}

ImgSensorData ReprojectionProcessor::process(sensor_msgs::msg::Image::ConstSharedPtr img)
{
  cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(img, img->encoding);
  auto remapped_img = cv_bridge::CvImage();

  cv::remap(
    cv_img->image,
    remapped_img.image,
    map_x,
    map_y,
    cv::INTER_LINEAR
  );

  sensor_msgs::msg::CameraInfo info;
  info.header = img->header;
  info.height = remapped_img.image.rows;
  info.width = remapped_img.image.cols;

  // set intrinsics
  std::fill(std::begin(info.k), std::end(info.k), 0.0);
  info.k[0] = to->intrinsics[0];
  info.k[2] = to->intrinsics[2];
  info.k[5] = to->intrinsics[1];
  info.k[6] = to->intrinsics[3];

  // set distortion
  size_t distortion_count = 0;
  switch (to->lensmodel.type) {
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
      throw std::logic_error("Reprojection Processor accepted incompatible lean lens model");
  }

  for (size_t idx = 0; idx < distortion_count; idx++) {
    info.d[idx] = to->intrinsics[4 + idx];
  }

  return {
    info,
    remapped_img.toImageMsg()
  };
}

}
