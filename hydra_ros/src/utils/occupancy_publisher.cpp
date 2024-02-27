/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/utils/occupancy_publisher.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/hydra_config.h>
#include <nav_msgs/OccupancyGrid.h>

namespace hydra {

using voxblox::Layer;

template <typename T>
float getDistance(const T& voxel);

template <>
float getDistance<voxblox::TsdfVoxel>(const voxblox::TsdfVoxel& voxel) {
  return voxel.distance;
}

template <>
float getDistance<places::GvdVoxel>(const places::GvdVoxel& voxel) {
  return voxel.distance;
}

template <typename T>
bool isObserved(const T& voxel, float min_weight);

template <>
bool isObserved<voxblox::TsdfVoxel>(const voxblox::TsdfVoxel& voxel, float min_weight) {
  return voxel.weight >= min_weight;
}

template <>
bool isObserved<places::GvdVoxel>(const places::GvdVoxel& voxel, float) {
  return voxel.observed;
}

template <typename T>
void fillOccupancy(const OccupancyPublisher::Config& config,
                   const Layer<T>& layer,
                   const Eigen::Isometry3d& world_T_sensor,
                   nav_msgs::OccupancyGrid& msg) {
  auto height = config.slice_height;
  if (config.use_relative_height) {
    height += world_T_sensor.translation().z();
  }

  const voxblox::Point slice_pos(0, 0, height);
  const auto slice_index = layer.computeBlockIndexFromCoordinates(slice_pos);
  const auto origin =
      voxblox::getOriginPointFromGridIndex(slice_index, layer.block_size());
  const auto grid_index = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
      slice_pos - origin, layer.voxel_size_inv());

  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  voxblox::BlockIndexList matching_blocks;
  for (const auto& idx : blocks) {
    if (idx.z() == slice_index.z()) {
      matching_blocks.push_back(idx);
    }
  }

  Eigen::Vector2f x_min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector2f x_max =
      Eigen::Vector2f::Constant(std::numeric_limits<float>::lowest());

  for (const auto& idx : matching_blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    const auto lower = block.origin();
    const auto upper = lower + Eigen::Vector3f::Constant(block.block_size());
    x_min = x_min.array().min(lower.template head<2>().array());
    x_max = x_max.array().max(upper.template head<2>().array());
  }

  const Eigen::Vector2f dims = (x_max - x_min) / layer.voxel_size();
  msg.info.resolution = layer.voxel_size();
  msg.info.width = std::ceil(dims.x() / layer.voxel_size());
  msg.info.height = std::ceil(dims.y() / layer.voxel_size());
  msg.info.origin.position.x = x_min.x();
  msg.info.origin.position.y = x_min.y();
  msg.info.origin.position.z = height;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(msg.info.width * msg.info.height);
  for (const auto& idx : matching_blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t x = 0; x < block.voxels_per_side(); ++x) {
      for (size_t y = 0; y < block.voxels_per_side(); ++y) {
        const voxblox::VoxelIndex voxel_index(x, y, grid_index.z());
        const auto& voxel = block.getVoxelByVoxelIndex(voxel_index);
        const Eigen::Vector3f pos = block.computeCoordinatesFromVoxelIndex(voxel_index);
        const auto rel_pos = pos.head<2>() - x_min;
        // pos is center point, so we want floor
        const auto r = std::floor(rel_pos.y() / layer.voxel_size());
        const auto c = std::floor(rel_pos.x() / layer.voxel_size());
        const size_t index = r * msg.info.width + c;
        if (!isObserved(voxel, config.min_observation_weight)) {
          msg.data[index] = -1;
          continue;
        }

        const auto occupied = getDistance(voxel) < config.min_distance;
        msg.data[index] = occupied ? 100 : 0;
      }
    }
  }
}

void declare_config(OccupancyPublisher::Config& config) {
  using namespace config;
  name("OccupancyPublisher::Config");
  field(config.use_relative_height, "use_relative_height");
  field(config.slice_height, "slice_height", "m");
  field(config.min_observation_weight, "min_observation_weight");
  field(config.min_distance, "min_distance");
}

OccupancyPublisher::OccupancyPublisher(const Config& config, const ros::NodeHandle& nh)
    : config(config::checkValid(config)),
      nh_(nh),
      pub_(nh_.advertise<nav_msgs::OccupancyGrid>("occupancy", 1, true)) {}

OccupancyPublisher::~OccupancyPublisher() {}

void OccupancyPublisher::publishTsdf(uint64_t timestamp_ns,
                                     const Eigen::Isometry3d& world_T_sensor,
                                     const Layer<voxblox::TsdfVoxel>& tsdf) const {
  nav_msgs::OccupancyGrid msg;
  msg.header.frame_id = HydraConfig::instance().getFrames().map;
  msg.header.stamp.fromNSec(timestamp_ns);

  msg.info.map_load_time = msg.header.stamp;

  fillOccupancy(config, tsdf, world_T_sensor, msg);
  pub_.publish(msg);
}

void OccupancyPublisher::publishGvd(uint64_t timestamp_ns,
                                    const Eigen::Isometry3d& world_T_sensor,
                                    const Layer<places::GvdVoxel>& gvd) const {
  nav_msgs::OccupancyGrid msg;
  msg.header.frame_id = HydraConfig::instance().getFrames().map;
  msg.header.stamp.fromNSec(timestamp_ns);

  msg.info.map_load_time = msg.header.stamp;

  fillOccupancy(config, gvd, world_T_sensor, msg);
  pub_.publish(msg);
}

TsdfOccupancyPublisher::TsdfOccupancyPublisher(const Config& config)
    : pub_(OccupancyPublisher(config.extraction, ros::NodeHandle(config.ns))) {}

GvdOccupancyPublisher::GvdOccupancyPublisher(const Config& config)
    : pub_(OccupancyPublisher(config.extraction, ros::NodeHandle(config.ns))) {}

void TsdfOccupancyPublisher::call(uint64_t timestamp_ns,
                                  const Eigen::Isometry3d& world_T_sensor,
                                  const voxblox::Layer<voxblox::TsdfVoxel>& tsdf,
                                  const ReconstructionOutput&) const {
  pub_.publishTsdf(timestamp_ns, world_T_sensor, tsdf);
}

void GvdOccupancyPublisher::call(uint64_t timestamp_ns,
                                 const Eigen::Isometry3f& world_T_body,
                                 const GvdPlaceExtractor::GvdLayer& gvd,
                                 const places::GraphExtractorInterface*) const {
  pub_.publishGvd(timestamp_ns, world_T_body.cast<double>(), gvd);
}

void declare_config(GvdOccupancyPublisher::Config& config) {
  using namespace config;
  name("GvdOccupancyPublisher::Config");
  field(config.ns, "ns");
  field(config.extraction, "extraction");
}

void declare_config(TsdfOccupancyPublisher::Config& config) {
  using namespace config;
  name("TsdfOccupancyPublisher::Config");
  field(config.ns, "ns");
  field(config.extraction, "extraction");
}

}  // namespace hydra
