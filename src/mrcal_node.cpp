/// @file mrcal_node.cpp
/// @brief Provides a node that loads mrcal intrinsics and reprojects the image

#include "mrcal_ros/mrcal_image_processing.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <filesystem>
#include <memory>
#include <mrcal/mrcal-types.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>

extern "C" {
#include <mrcal/mrcal.h>
}

namespace mrcal_ros {

class MrCal : public rclcpp::Node {
public:
  MrCal(const rclcpp::NodeOptions &options) : Node("mrcal", options) {
    const auto make_desc = [](const std::string_view description) {
      rcl_interfaces::msg::ParameterDescriptor param_desc{};
      param_desc.description = description;
      return param_desc;
    };

    this->declare_parameter<std::string>(
        "intrinsics", "", make_desc("Filepath to the mrcal intrinsics file"));

    this->declare_parameter<std::string>(
        "reprojection_model", "",
        make_desc("Filepath of intrinsic model to reproject to"));

    const auto param_to_model = [this](const std::string_view name) {
      const std::filesystem::path path{
          this->get_parameter(std::string{name}).as_string()};

      if (path.empty() || !std::filesystem::exists(path)) {
        const std::string msg =
            std::string{name} + " does not exist at: " + path.generic_string();
        throw std::runtime_error(msg);
      }

      auto camera_model = std::unique_ptr<mrcal_cameramodel_t>(
          mrcal_read_cameramodel_file(path.c_str()));

      if (!camera_model) {
        const std::string msg =
            "Could not read camera model file: " + path.generic_string();
        throw std::runtime_error(msg);
      }

      return camera_model;
    };

    const auto intrinsic_model = param_to_model("intrinsics");

    // if the intrinsics are the pinhole, 5, or 8 parameter opencv model they
    // are supported directly and no reprojection is necesssary.
    if (intrinsic_model->lensmodel.type == MRCAL_LENSMODEL_OPENCV5 ||
        intrinsic_model->lensmodel.type == MRCAL_LENSMODEL_OPENCV8) {
      // we can just republish the image with an accompanying camera info topic
      // containing the intrinsics
    } else {
      // if the intrinsics are any other model we need to reproject to one of
      // the supported models
      if (this->get_parameter("reprojection_model").as_string().empty()) {
        throw std::runtime_error(
            "Need reprojection model since intrinsic type is not supported.");
      }
      const auto reprojection_model = param_to_model("reprojection_model");
    }

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          this->process_image(msg);
        });
  };

private:
  /// @brief Configured mrcal image processor
  std::unique_ptr<IMrCalProcessor> mrcal_processor;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  void process_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    const auto &[info, image] = mrcal_processor->process(msg);
    camera_info_pub_->publish(info);
    image_pub_->publish(image);
  }
};

} // namespace mrcal_ros

RCLCPP_COMPONENTS_REGISTER_NODE(mrcal_ros::MrCal);
