#include "hydra_ros/frontend/frontier_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <hydra/common/global_info.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/drawing.h>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra {

using geometry_msgs::msg::Point;
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
                              const std::vector<Frontier>& frontiers) const {
  std_msgs::msg::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp = rclcpp::Time(timestamp_ns);

  pubs_.publish("rayfronts_viz", header, [&]() -> MarkerArray {
    return drawRayFronts(frontiers, "rayfronts");
  });
}

void FrontierVisualizer::drawRayFronts(const std::vector<Frontier>& frontiers,
                                       const std::string& ns) const {
  MarkerArray marker_array;
  int id = 0;

  for (const auto& f : frontiers) {
    const Eigen::Vector3d& origin = f.center;

    for (const auto& rf : f.rayfronts) {
      Marker arrow;
      arrow.header.frame_id = frame_id;
      arrow.header.stamp = rclcpp::Clock().now();
      arrow.ns = ns;
      arrow.id = id++;
      arrow.type = Marker::ARROW;
      arrow.action = Marker::ADD;

      Point p_start;
      p_start.x = origin.x();
      p_start.y = origin.y();
      p_start.z = origin.z();

      geometry_msgs::msg::Point p_end;
      Eigen::Vector3d dir = rf.direction.normalized() * config.arrow_length;
      p_end.x = origin.x() + dir.x();
      p_end.y = origin.y() + dir.y();
      p_end.z = origin.z() + dir.z();

      arrow.points.push_back(p_start);
      arrow.points.push_back(p_end);

      arrow.scale.x = 0.02;
      arrow.scale.y = 0.04;
      arrow.scale.z = 0.1;

      auto color = label_colormap_(rf.semantic_label);
      arrow.color.r = color.r;
      arrow.color.g = color.g;
      arrow.color.b = color.b;
      arrow.color.a = color.a;

      marker_array.markers.push_back(arrow);
    }
  }
}
}  // namespace hydra