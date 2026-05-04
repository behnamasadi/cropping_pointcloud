// Crop an incoming PointCloud2 against an axis-aligned bounding box and
// optionally extract the dominant non-planar object via RANSAC plane removal.
//
// Bounds (x_min/x_max/y_min/y_max/z_min/z_max), the input topic, and the
// frame_id are exposed as ROS 2 parameters. All six bounds are live-tunable;
// changes take effect immediately via the on-set-parameters callback.

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

class CroppingPointCloud : public rclcpp::Node
{
public:
  CroppingPointCloud()
  : rclcpp::Node("cropping_pointcloud")
  {
    declare_parameter<std::string>("input_topic", "camera/rgb/points");
    declare_parameter<std::string>("frame_id", "");   // empty → keep input frame
    declare_parameter<bool>("extract_object", true);
    declare_parameter<double>("plane_distance_threshold", 0.02);
    declare_parameter<int>("plane_max_iterations", 100);

    declare_parameter<double>("x_min", -0.44);
    declare_parameter<double>("x_max",  0.30);
    declare_parameter<double>("y_min", -1.00);
    declare_parameter<double>("y_max",  0.24);
    declare_parameter<double>("z_min", -1.00);
    declare_parameter<double>("z_max",  1.08);

    refresh_bounds_from_parameters();

    const auto input_topic = get_parameter("input_topic").as_string();

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { on_cloud(msg); });

    pub_cropped_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cropped_cloud", rclcpp::SensorDataQoS());
    pub_object_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "object_cloud", rclcpp::SensorDataQoS());

    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_parameter_change(params);
      });

    RCLCPP_INFO(get_logger(),
      "subscribed to '%s'; publishing /cropped_cloud and /object_cloud",
      input_topic.c_str());
  }

private:
  void refresh_bounds_from_parameters()
  {
    x_min_ = get_parameter("x_min").as_double();
    x_max_ = get_parameter("x_max").as_double();
    y_min_ = get_parameter("y_min").as_double();
    y_max_ = get_parameter("y_max").as_double();
    z_min_ = get_parameter("z_min").as_double();
    z_max_ = get_parameter("z_max").as_double();
  }

  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      const auto & name = p.get_name();
      if      (name == "x_min") x_min_ = p.as_double();
      else if (name == "x_max") x_max_ = p.as_double();
      else if (name == "y_min") y_min_ = p.as_double();
      else if (name == "y_max") y_max_ = p.as_double();
      else if (name == "z_min") z_min_ = p.as_double();
      else if (name == "z_max") z_max_ = p.as_double();
      else if (name == "extract_object" || name == "frame_id" ||
               name == "plane_distance_threshold" ||
               name == "plane_max_iterations") {
        // handled at use-site via get_parameter
      }
    }
    RCLCPP_INFO(get_logger(),
      "bounds: x[%.3f, %.3f] y[%.3f, %.3f] z[%.3f, %.3f]",
      x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
    return result;
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    auto input = std::make_shared<CloudT>();
    pcl::fromROSMsg(*msg, *input);

    // Axis-aligned crop: pcl::CropBox is the right tool (the original used
    // ConvexHull + CropHull, which is overkill and had a vertex typo).
    auto cropped = std::make_shared<CloudT>();
    {
      pcl::CropBox<PointT> box;
      box.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0f));
      box.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0f));
      box.setInputCloud(input);
      box.filter(*cropped);
    }

    publish_cloud(*cropped, msg->header, pub_cropped_);

    if (get_parameter("extract_object").as_bool() && !cropped->empty()) {
      CloudT object;
      if (extract_non_planar(*cropped, object)) {
        publish_cloud(object, msg->header, pub_object_);
      }
    }
  }

  bool extract_non_planar(const CloudT & input, CloudT & output)
  {
    pcl::SACSegmentation<PointT> sac;
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setMaxIterations(get_parameter("plane_max_iterations").as_int());
    sac.setDistanceThreshold(
      get_parameter("plane_distance_threshold").as_double());

    auto inliers = std::make_shared<pcl::PointIndices>();
    auto coeffs  = std::make_shared<pcl::ModelCoefficients>();
    sac.setInputCloud(input.makeShared());
    sac.segment(*inliers, *coeffs);

    if (inliers->indices.empty()) return false;

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);   // keep everything that's NOT the plane
    extract.filter(output);
    return true;
  }

  void publish_cloud(
    const CloudT & cloud,
    const std_msgs::msg::Header & input_header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    const auto frame_override = get_parameter("frame_id").as_string();
    msg.header = input_header;
    if (!frame_override.empty()) msg.header.frame_id = frame_override;
    pub->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cropped_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CroppingPointCloud>());
  rclcpp::shutdown();
  return 0;
}
