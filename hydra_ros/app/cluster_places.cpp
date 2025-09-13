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
#include <config_utilities/types/enum.h>
#include <config_utilities_ros/ros_dynamic_config_server.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <hydra/openset/embedding_distances.h>
#include <hydra/rooms/room_finder.h>
#include <ianvs/node_init.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/scene_graph_layer.h>

#include <rclcpp/rclcpp.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

using hydra::FeatureMap;
using hydra::FeatureVector;
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

enum class FeatureType { CLIP = 0, SENTENCE = 1, BOTH = 2 };

struct Config {
  std::string src_path;
  std::string dst_path;
  FeatureType feature_type = FeatureType::BOTH;
  config::VirtualConfig<hydra::EmbeddingDistance> distance_metric;
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
  enum_field(config.feature_type,
             "feature_type",
             {{FeatureType::CLIP, "CLIP"},
              {FeatureType::SENTENCE, "SENTENCE"},
              {FeatureType::BOTH, "BOTH"}});
  field(config.distance_metric, "distance_metric");
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
    if (!graph_->hasLayer(config.places_layer)) {
      LOG(ERROR) << "Graph does not have a layer named '" << config.places_layer
                 << "', cannot cluster places.";
      return;
    }

    // computeFeatures.
    const bool features_updated = computeFeatures(config);
    if (features_updated || !prev_config_.distance_metric != config.distance_metric) {
      computeDistances(config);
    }

    // Cluster.
    cluster(config);

    // Publish.
    sender_.sendGraph(*graph_, rclcpp::Time(0));

    // Save.
    save(config);
    prev_config_ = config;
  }

 private:
  const hydra::DsgSender sender_;
  DynamicSceneGraph::Ptr graph_;
  Config prev_config_;
  FeatureMap<int> node_features_;

  bool computeFeatures(const Config& config) {
    if (!node_features_.empty() && config.feature_type == prev_config_.feature_type) {
      return false;
    }
    node_features_.clear();
    size_t num_invalid = 0;

    // Extract the feature map.
    const auto& meta_data = graph_->metadata.get()["features"];
    for (const auto& [key, value] : meta_data.items()) {
      const int id = std::stoi(key);
      if (!value.contains("clip_feature") || value["clip_feature"].is_null() ||
          !value.contains("sentence_embedding_feature") ||
          value["sentence_embedding_feature"].is_null()) {
        num_invalid++;
        continue;
      }

      const auto clip_vec = value["clip_feature"].get<std::vector<float>>();
      const auto sentence_vec =
          value["sentence_embedding_feature"].get<std::vector<float>>();
      FeatureVector feature;
      switch (config.feature_type) {
        case FeatureType::CLIP:
          feature = Eigen::Map<const FeatureVector>(clip_vec.data(), clip_vec.size());
          break;
        case FeatureType::SENTENCE:
          feature =
              Eigen::Map<const FeatureVector>(sentence_vec.data(), sentence_vec.size());
          break;
        case FeatureType::BOTH: {
          feature = FeatureVector(clip_vec.size() + sentence_vec.size());
          feature << Eigen::Map<const FeatureVector>(clip_vec.data(), clip_vec.size()),
              Eigen::Map<const FeatureVector>(sentence_vec.data(), sentence_vec.size());
          break;
        }
        default:
          LOG(ERROR) << "Unhandled feature type.";
          return false;
      }
      node_features_[id] = feature;
    }

    // Remap invalid features to zero vectors.
    const int feature_dim = node_features_.begin()->second.size();
    LOG(INFO) << "Read " << node_features_.size() << " features, " << num_invalid
              << " invalid. Feature dim: " << feature_dim;
    return true;
  }

  void computeDistances(const Config& config) {
    const auto metric = config.distance_metric.create();

    // Compute the edge similarities.
    size_t num_invalid = 0;
    const auto& places_layer = graph_->getLayer(config.places_layer);
    for (auto& [_, edge] : places_layer.edges()) {
      const auto& source_node = places_layer.getNode(edge.source);
      const auto& source_attrs = source_node.attributes<TraversabilityNodeAttributes>();
      const auto source_feature_it =
          node_features_.find(source_attrs.cognition_labels.begin()->first);
      if (source_feature_it == node_features_.end()) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      const auto& target_node = places_layer.getNode(edge.target);
      const auto& target_attrs = target_node.attributes<TraversabilityNodeAttributes>();
      const auto target_feature_it =
          node_features_.find(target_attrs.cognition_labels.begin()->first);
      if (target_feature_it == node_features_.end()) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      // Compute the distance.
      const float dist =
          metric->dist(source_feature_it->second, target_feature_it->second);
      edge.info->weight = dist;
      edge.info->weighted = true;
    }
    LOG(INFO) << "Computed distances for " << places_layer.edges().size() << " edges, "
              << num_invalid << " invalid.";
  }

  void cluster(const Config& config) {
    hydra::RoomFinder room_finder(config.room_finder);
    // auto rooms =
    // room_finder.findRooms(*graph_->getLayer(config.places_layer).clone());
    auto rooms = room_finder.findRooms(graph_->getLayer(config.places_layer));
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
    if (config.src_path == prev_config_.src_path && graph_) {
      return;
    }
    graph_ = spark_dsg::DynamicSceneGraph::load(config.src_path);
    if (!graph_) {
      LOG(ERROR) << "Failed to load graph from '" << config.src_path << "'.";
      return;
    }
    LOG(INFO) << "Loaded graph from '" << config.src_path << "'.";
    node_features_.clear();
  }

  void save(const Config& config) {
    if (config.dst_path == prev_config_.dst_path) {
      return;
    }
    graph_->save(config.dst_path);
    LOG(INFO) << "Saved clustered graph to '" << config.dst_path << "'.";
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
  clusterer.process(config.get());

  // Spin.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh.as<rclcpp::node_interfaces::NodeBaseInterface>());
  const config::RosDynamicConfigServer server(nh.node());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
