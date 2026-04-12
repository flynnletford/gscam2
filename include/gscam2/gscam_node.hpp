#ifndef __GSCAM_GSCAM_H
#define __GSCAM_GSCAM_H

#include "rclcpp/rclcpp.hpp"

#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

extern "C" {
#include <gst/gst.h>
}

namespace gscam2
{

class GSCamNode : public rclcpp::Node
{
  // Hide implementation
  class impl;
  std::unique_ptr<impl> pImpl_;
  rclcpp::OnShutdownCallbackHandle on_shutdown_handle_;

  void validate_parameters();

public:
  explicit GSCamNode(const rclcpp::NodeOptions & options);

  ~GSCamNode() override;

private:
  GstElement* tcambin_ = nullptr;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter>& params);
};

}

#endif // ifndef __GSCAM_GSCAM_H
