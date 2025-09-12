/**
 * Offline Script to load a scene graph, cluster the places based on their similarity
 * labels, and visualize the result.
 */

#include <config_utilities/config.h>
#include <config_utilities/dynamic_config.h>
#include <config_utilities/external_registry.h>
#include <config_utilities/logging/log_to_glog.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/settings.h>
#include <config_utilities_ros/ros_dynamic_config_server.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra/rooms/room_finder.h>
#include <ianvs/node_init.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/scene_graph_layer.h>

#include <rclcpp/rclcpp.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraphLayer;
using spark_dsg::TraversabilityNodeAttributes;

struct NodeConfig {
  std::vector<std::string> external_library_paths;
  bool interactive = true;
};

struct Config {
  std::string src_path;
  std::string dst_path;
  std::string places_layer = DsgLayers::TRAVERSABILITY;
  hydra::RoomFinderConfig room_finder;
};

void declare_config(NodeConfig& config) {
  using namespace config;
  name("ClusterPlaces::NodeConfig");
  field(config.external_library_paths, "external_library_paths");
  field(config.interactive, "interactive");
}

void declare_config(Config& config) {
  using namespace config;
  name("ClusterPlaces::Config");
  field(config.src_path, "src_path");
  field(config.dst_path, "dst_path");
  field(config.places_layer, "places_layer");
  field(config.room_finder, "room_finder");
}

class Clusterer {
 public:
  Clusterer(ianvs::NodeHandle nh)
      : sender_(nh, "world", "publish_dsg", true, 0.0, true) {}

  void process(const Config& config) {
    LOG(INFO) << "Processing:\n" << config::toString(config);
    // Load.
    load(config);
    if (!graph_) {
      LOG(WARNING) << "No graph loaded, cannot process";
      return;
    }

    // Cluster.
    cluster(config);

    // Publish.
    sender_.sendGraph(*graph_, rclcpp::Time(0));

    // Save.
    save(config);
  }

 private:
  const hydra::DsgSender sender_;
  DynamicSceneGraph::Ptr graph_;
  std::string prev_src_;
  std::string prev_dst_;

  void preprocess(const Config& config) {
    // Compute the edge similarities.
  }

  void cluster(const Config& config) {
    const auto places_layer = graph_->findLayer(config.places_layer);
    if (!places_layer) {
      LOG(WARNING) << "Graph does not contain a layer named '" << config.places_layer
                   << "', cannot cluster places.";
      return;
    }
    auto places_clone = places_layer->clone();

    // Find Rooms.
    hydra::RoomFinder room_finder(config.room_finder);
    auto rooms = room_finder.findRooms(*places_clone);
    rewriteRooms(rooms.get(), *graph_);
    room_finder.addRoomPlaceEdges(*graph_, config.places_layer);
  }

  void rewriteRooms(const SceneGraphLayer* new_rooms, DynamicSceneGraph& graph) const {
    std::vector<NodeId> to_remove;
    const auto& prev_rooms = graph.getLayer(DsgLayers::ROOMS);
    for (const auto& id_node_pair : prev_rooms.nodes()) {
      to_remove.push_back(id_node_pair.first);
    }

    for (const auto node_id : to_remove) {
      graph.removeNode(node_id);
    }

    if (!new_rooms) {
      return;
    }

    for (auto&& [id, node] : new_rooms->nodes()) {
      graph.emplaceNode(DsgLayers::ROOMS, id, node->attributes().clone());
    }

    for (const auto& id_edge_pair : new_rooms->edges()) {
      const auto& edge = id_edge_pair.second;
      graph.insertEdge(edge.source, edge.target, edge.info->clone());
    }
  }

  void load(const Config& config) {
    if (config.src_path != prev_src_) {
      prev_src_ = config.src_path;
      graph_ = spark_dsg::DynamicSceneGraph::load(config.src_path);
      if (!graph_) {
        LOG(ERROR) << "Failed to load graph from '" << config.src_path << "'.";
        return;
      }
      LOG(INFO) << "Loaded graph from '" << config.src_path << "'.";
      preprocess(config);
    }
  }

  void save(const Config& config) {
    if (config.dst_path != prev_dst_) {
      prev_dst_ = config.dst_path;
      graph_->save(config.dst_path);
      LOG(INFO) << "Saved clustered graph to '" << config.dst_path << "'.";
    }
  }
};

int main(int argc, char** argv) {
  config::initContext(argc, argv, true);
  config::setConfigSettingsFromContext();

  // Node
  [[maybe_unused]] const auto guard =
      ianvs::init_node(argc, argv, "cluster_places_node");
  auto nh = ianvs::NodeHandle::this_node("~");

  // Glog.
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  config::Settings().setLogger("glog");

  // Setup config utilities.
  const auto node_settings = config::fromContext<NodeConfig>();
  [[maybe_unused]] const auto plugins_guard =
      config::loadExternalFactories(node_settings.external_library_paths);
  LOG(INFO) << "Loaded node settings:\n" << config::toString(node_settings);

  // ================= Start processing =================
  // Load config.
  Clusterer clusterer(nh);
  const Config initial_config = config::fromContext<Config>();

  if (!node_settings.interactive) {
    clusterer.process(initial_config);
    LOG(INFO) << "Done. Exiting.";
    rclcpp::shutdown();
    return 0;
  }

  // Run interactively.
  config::DynamicConfig<Config> config("cluster_places", initial_config);
  config.setCallback([&clusterer, &config]() { clusterer.process(config.get()); });
  const config::RosDynamicConfigServer server(nh.node());
  clusterer.process(config.get());

  // Spin.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh.as<rclcpp::node_interfaces::NodeBaseInterface>());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
