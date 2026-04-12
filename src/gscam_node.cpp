#include "gscam2/gscam_node.hpp"

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "camera_info_manager/camera_info_manager.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "tcam-property-1.0.h"

namespace gscam2
{

//=============================================================================
// Parameters
//=============================================================================

struct GSCamContext
{
  std::string gst_plugin_path_;
  std::string gscam_config_;
  bool sync_sink_{};
  bool preroll_{};
  bool use_gst_timestamps_{};
  std::string image_encoding_;
  std::string camera_info_url_;
  std::string camera_name_;
  std::string frame_id_;
  int64_t skip_{};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameters_callback_handle_;
};

//=============================================================================
// impl
//=============================================================================

class GSCamNode::impl
{
  rclcpp::Node * node_;

  camera_info_manager::CameraInfoManager camera_info_manager_;

  GstElement * pipeline_;
  GstElement * sink_;

  GstElement * tcambin_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr control_param_callback_;

  std::thread pipeline_thread_;
  std::atomic<bool> stop_signal_;
  std::atomic<bool> shutdown_signal_;

  int width_, height_;
  GstClockTime time_offset_;
  int64_t skip_count_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  bool create_pipeline();
  void delete_pipeline();
  void process_frame();

public:
  GSCamContext cxt_;

  explicit impl(rclcpp::Node * node)
  : node_(node),
    camera_info_manager_(node),
    pipeline_(nullptr),
    sink_(nullptr),
    tcambin_(nullptr),
    stop_signal_(false),
    shutdown_signal_(false),
    width_(0),
    height_(0),
    time_offset_(0),
    skip_count_(0)
  {}

  ~impl() { shutdown(); }

  void shutdown();
  void restart();
};

//=============================================================================
// Pipeline creation
//=============================================================================

bool GSCamNode::impl::create_pipeline()
{
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  GError *error = nullptr;
  pipeline_ = gst_parse_launch(cxt_.gscam_config_.c_str(), &error);

  if (!pipeline_) {
    RCLCPP_FATAL(node_->get_logger(), "%s", error->message);
    return false;
  }

  tcambin_ = gst_bin_get_by_name(GST_BIN(pipeline_), "zoom_camera_pipeline");

  if (!tcambin_) {
    RCLCPP_ERROR(node_->get_logger(),
      "TCAM element 'zoom_camera_pipeline' not found in pipeline");
    return false;
  }

  // Declare params
  node_->declare_parameter("zoom", 0);
  node_->declare_parameter("focus", 10000);
  node_->declare_parameter("iris", 255);

  // ============================================================
  // Control callback
  // ============================================================
  if (!control_param_callback_) {
    control_param_callback_ =
      node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;

          if (!tcambin_) {
            RCLCPP_ERROR(node_->get_logger(), "tcambin not available");
            result.successful = false;
            return result;
          }

          for (const auto &param : params)
          {
            try
            {
              GError *err = nullptr;

              if (param.get_name() == "zoom")
              {
                int val = param.as_int();

                tcam_property_provider_set_tcam_integer(
                  TCAM_PROPERTY_PROVIDER(tcambin_),
                  "Zoom",
                  val,
                  &err);

                if (err) {
                  RCLCPP_ERROR(node_->get_logger(), "Zoom failed: %s", err->message);
                  g_error_free(err);
                  result.successful = false;
                } else {
                  RCLCPP_INFO(node_->get_logger(), "Zoom set to %d", val);
                }
              }

              else if (param.get_name() == "focus")
              {
                int val = param.as_int();

                tcam_property_provider_set_tcam_integer(
                  TCAM_PROPERTY_PROVIDER(tcambin_),
                  "Focus",
                  val,
                  &err);

                if (err) {
                  RCLCPP_ERROR(node_->get_logger(), "Focus failed: %s", err->message);
                  g_error_free(err);
                  result.successful = false;
                } else {
                  RCLCPP_INFO(node_->get_logger(), "Focus set to %d", val);
                }
              }

              else if (param.get_name() == "iris")
              {
                int val = param.as_int();

                tcam_property_provider_set_tcam_integer(
                  TCAM_PROPERTY_PROVIDER(tcambin_),
                  "Iris",
                  val,
                  &err);

                if (err) {
                  RCLCPP_ERROR(node_->get_logger(), "Iris failed: %s", err->message);
                  g_error_free(err);
                  result.successful = false;
                } else {
                  RCLCPP_INFO(node_->get_logger(), "Iris set to %d", val);
                }
              }
            }
            catch (const std::exception &e)
            {
              RCLCPP_ERROR(node_->get_logger(), "Param error: %s", e.what());
              result.successful = false;
            }
          }

          return result;
        });
  }

  // (rest of your pipeline setup stays unchanged...)
  // sink creation, caps, linking, etc.

  return true;
}

//=============================================================================
// shutdown/restart placeholders
//=============================================================================

void GSCamNode::impl::shutdown() {}
void GSCamNode::impl::restart() {}

} // namespace gscam2