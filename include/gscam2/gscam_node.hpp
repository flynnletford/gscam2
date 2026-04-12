#ifndef __GSCAM_GSCAM_H
#define __GSCAM_GSCAM_H

#include "rclcpp/rclcpp.hpp"

#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "std_srvs/srv/set_bool.hpp"

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

  // Service to set AutoIris
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_auto_iris_srv_;
  void handle_set_auto_iris(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std srvs::srv::SetBool::Response> response);
};

}

#endif // ifndef __GSCAM_GSCAM_H
