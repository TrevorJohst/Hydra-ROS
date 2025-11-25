#include "hydra_ros/frontend/frontier_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <hydra/common/global_info.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/drawing.h>

#include <boost/range/join.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace hydra {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(FrontierVisualizer::Config& config) {
  using namespace config;
  name("FrontierVisualizerConfig");
  field(config.ns, "ns");
  field(config.label_colormap, "label_colormap");
}

FrontierVisualizer::FrontierVisualizer(const Config& config)
    : config(config),
      nh_(ianvs::NodeHandle::this_node(config.ns)),
      pubs_(nh_),
      label_colormap_(config.label_colormap) {}

std::string FrontierVisualizer::printInfo() const { return config::toString(config); }

void FrontierVisualizer::call(uint64_t timestamp_ns,
                              const std::vector<Frontier>& frontiers,
                              const std::vector<Frontier>& archived_frontiers) const {
  std_msgs::msg::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp = rclcpp::Time(timestamp_ns);

  pubs_.publish("frontier_viz", header, [&]() -> MarkerArray {
    return drawFrontiers(frontiers, archived_frontiers, "frontiers");
  });

  pubs_.publish("rayfronts_viz", header, [&]() -> MarkerArray {
    return drawRayFronts(frontiers, archived_frontiers, "rayfronts");
  });
}

MarkerArray FrontierVisualizer::drawFrontiers(
    const std::vector<Frontier>& frontiers,
    const std::vector<Frontier>& archived_frontiers,
    const std::string& ns) const {
  MarkerArray marker_array;
  int id = 0;

  for (const auto& f : boost::join(frontiers, archived_frontiers)) {
    Marker base_circle;
    base_circle.ns = ns;
    base_circle.id = id++;
    base_circle.type = Marker::SPHERE;
    base_circle.action = Marker::ADD;

    geometry_msgs::msg::Point p_start;
    p_start.x = f.center.x();
    p_start.y = f.center.y();
    p_start.z = f.center.z();

    base_circle.pose.position = p_start;
    base_circle.pose.orientation.w = 1.0;
    base_circle.scale.x = 0.1;
    base_circle.scale.y = 0.1;
    base_circle.scale.z = 0.1;

    base_circle.color.r = 0.0f;
    base_circle.color.g = 0.0f;
    base_circle.color.b = 0.0f;
    base_circle.color.a = 1.0f;

    marker_array.markers.push_back(base_circle);
  }
  return marker_array;
}

MarkerArray FrontierVisualizer::drawRayFronts(
    const std::vector<Frontier>& frontiers,
    const std::vector<Frontier>& archived_frontiers,
    const std::string& ns) const {
  MarkerArray marker_array;
  int id = 0;

  for (const auto& f : boost::join(frontiers, archived_frontiers)) {
    const Eigen::Vector3f& origin = f.center.cast<float>();

    for (const auto& rf : f.rayfronts) {
      Marker arrow;
      arrow.ns = ns;
      arrow.id = id++;
      arrow.type = Marker::ARROW;
      arrow.action = Marker::ADD;

      geometry_msgs::msg::Point p_start;
      p_start.x = origin.x();
      p_start.y = origin.y();
      p_start.z = origin.z();

      geometry_msgs::msg::Point p_end;
      Eigen::Vector3f dir = rf.direction.normalized() * config.arrow_length;
      p_end.x = origin.x() + dir.x();
      p_end.y = origin.y() + dir.y();
      p_end.z = origin.z() + dir.z();

      arrow.points.push_back(p_start);
      arrow.points.push_back(p_end);

      arrow.scale.x = 0.05;  // Shaft diameter
      arrow.scale.y = 0.15;  // Head diameter
      arrow.scale.z = 0.20;  // Head length

      arrow.color =
          hydra::visualizer::makeColorMsg(label_colormap_(rf.semantic_label), 1.0f);
      marker_array.markers.push_back(arrow);
    }
  }
  return marker_array;
}
}  // namespace hydra