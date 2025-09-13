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
    if (!load(config)) {
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
    LOG(INFO) << "Published clustered graph.";

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
      FeatureVector feature;
      switch (config.feature_type) {
        case FeatureType::CLIP:
          feature = getClipFeature(value);
          if (feature.size() == 0) {
            num_invalid++;
            continue;
          }
          break;
        case FeatureType::SENTENCE:
          feature = getSentenceFeature(value);
          if (feature.size() == 0) {
            num_invalid++;
            continue;
          }
          break;
        case FeatureType::BOTH: {
          const auto clip_feature = getClipFeature(value);
          const auto sentence_feature = getSentenceFeature(value);
          if (clip_feature.size() == 0 || sentence_feature.size() == 0) {
            num_invalid++;
            continue;
          }
          feature.resize(clip_feature.size() + sentence_feature.size());
          feature << clip_feature, sentence_feature;
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
    LOG(INFO) << "Read " << (node_features_.size() + num_invalid)
              << " features, of which " << num_invalid
              << " invalid. Feature dim: " << feature_dim;
    return true;
  }

  FeatureVector getClipFeature(const nlohmann::json& meta) {
    if (!meta.contains("clip_feature") || meta["clip_feature"].is_null()) {
      return {};
    }
    const auto clip_vec = meta["clip_feature"].get<std::vector<float>>();
    if (clip_vec.empty()) {
      return {};
    }
    return Eigen::Map<const FeatureVector>(clip_vec.data(), clip_vec.size());
  }

  FeatureVector getSentenceFeature(const nlohmann::json& meta) {
    if (!meta.contains("sentence_embedding_feature") ||
        meta["sentence_embedding_feature"].is_null()) {
      return {};
    }
    const auto sentence_vec =
        meta["sentence_embedding_feature"].get<std::vector<float>>();
    if (sentence_vec.empty()) {
      return {};
    }
    return Eigen::Map<const FeatureVector>(sentence_vec.data(), sentence_vec.size());
  }

  void computeDistances(const Config& config) {
    const auto metric = config.distance_metric.create();
    const auto& layer = graph_->getLayer(config.places_layer);

    for (const auto& [_, node] : layer.nodes()) {
      node->attributes<TraversabilityNodeAttributes>().distance = 0.0;
    }

    // Compute the edge similarities.
    size_t num_invalid = 0;
    std::vector<float> distances;
    distances.reserve(layer.edges().size());
    for (auto& [_, edge] : layer.edges()) {
      edge.info->weighted = true;
      const auto& source_node = layer.getNode(edge.source);
      auto& source_attrs = source_node.attributes<TraversabilityNodeAttributes>();
      const auto source_feature_it =
          node_features_.find(source_attrs.cognition_labels.begin()->first);
      if (source_feature_it == node_features_.end()) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      const auto& target_node = layer.getNode(edge.target);
      auto& target_attrs = target_node.attributes<TraversabilityNodeAttributes>();
      const auto target_feature_it =
          node_features_.find(target_attrs.cognition_labels.begin()->first);
      if (target_feature_it == node_features_.end()) {
        edge.info->weight = 0.0f;
        num_invalid++;
        continue;
      }

      // Compute the similarities.
      const double score = 1.0 - 0.5 * metric->score(source_feature_it->second,
                                                     target_feature_it->second);
      edge.info->weight = score;
      source_attrs.distance = std::max(source_attrs.distance, score);
      target_attrs.distance = std::max(target_attrs.distance, score);
      distances.push_back(score);
    }

    // Compute statistics
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    double sum = 0.0;
    double sum_sq = 0.0;
    for (const auto& d : distances) {
      if (d < min_score) min_score = d;
      if (d > max_score) max_score = d;
      sum += d;
      sum_sq += d * d;
    }
    double mean = distances.empty() ? 0.0 : sum / distances.size();
    double stddev = 0.0;
    if (!distances.empty()) {
      stddev = std::sqrt(sum_sq / distances.size() - mean * mean);
    }

    LOG(INFO) << "Computed scores for " << layer.edges().size() << " edges, of which "
              << num_invalid << " invalid. "
              << "min: " << min_score << ", max: " << max_score << ", mean: " << mean
              << ", stddev: " << stddev;
  }

  void cluster(const Config& config) {
    hydra::RoomFinder room_finder(config.room_finder);
    auto rooms = room_finder.findRooms(*graph_->getLayer(config.places_layer).clone());
    // auto rooms = room_finder.findRooms(graph_->getLayer(config.places_layer));
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

  bool load(const Config& config) {
    if (config.src_path == prev_config_.src_path && graph_) {
      return true;
    }
    graph_ = spark_dsg::DynamicSceneGraph::load(config.src_path);
    if (!graph_) {
      LOG(ERROR) << "Failed to load graph from '" << config.src_path << "'.";
      return false;
    }
    LOG(INFO) << "Loaded graph from '" << config.src_path << "'.";

    if (!graph_->hasLayer(config.places_layer)) {
      LOG(ERROR) << "Graph does not have a layer named '" << config.places_layer
                 << "', cannot cluster places.";
      return false;
    }
    node_features_.clear();
    return true;
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
