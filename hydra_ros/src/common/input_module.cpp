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
#include "hydra_ros/common/input_module.h"

#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/common.h>
#include <hydra/common/hydra_config.h>

#include "hydra_ros/utils/lookup_tf.h"

namespace hydra {

InputModule::InputModule(const InputConfig& config, const OutputQueue::Ptr& queue)
    : config(config::checkValid(config)),
      nh_(ros::NodeHandle(config.ns)),
      queue_(queue) {
  buffer_.reset(new tf2_ros::Buffer(ros::Duration(config.tf_buffer_size_s)));
  tf_listener_.reset(new tf2_ros::TransformListener(*buffer_));

  receiver_ = config.receiver.create();
}

InputModule::~InputModule() { stop(); }

void InputModule::start() {
  receiver_->init();
  data_thread_.reset(new std::thread(&InputModule::dataSpin, this));
  LOG(INFO) << "[Hydra Input] started!";
}

void InputModule::stop() {
  should_shutdown_ = true;

  if (data_thread_) {
    VLOG(VLEVEL_TRACE) << "[Hydra Input] stopping pointcloud input thread";
    data_thread_->join();
    data_thread_.reset();
    VLOG(VLEVEL_TRACE) << "[Hydra Input] stopped pointcloud input thread";
  }

  VLOG(VLEVEL_TRACE) << "[Hydra Input] data queue: " << receiver_->queue.size();
}

void InputModule::save(const LogSetup&) {}

std::string InputModule::printInfo() const {
  std::stringstream ss;
  ss << config::toString(config);
  return ss.str();
}

void InputModule::dataSpin() {
  while (!should_shutdown_) {
    bool has_data = receiver_->queue.poll();
    if (!has_data) {
      continue;
    }

    const auto packet = receiver_->queue.pop();
    const auto curr_time = packet->timestamp_ns;
    VLOG(VLEVEL_TRACE) << "[ROS Input] popped input @ " << curr_time << " [ns]";

    // TODO(nathan) can refactor out and move to non-ros
    ros::Time curr_ros_time;
    curr_ros_time.fromNSec(curr_time);
    const auto pose_status = lookupTransform(*buffer_,
                                             curr_ros_time,
                                             HydraConfig::instance().getFrames().odom,
                                             HydraConfig::instance().getFrames().robot,
                                             5,  // max tries
                                             config.tf_wait_duration_s);
    if (!pose_status.is_valid) {
      LOG(WARNING) << "[ROS Input] dropping input @ " << curr_time
                   << " [ns] due to missing pose";
      continue;
    }

    ReconstructionInput::Ptr input(new ReconstructionInput());
    input->timestamp_ns = curr_time;
    input->sensor_input = packet;
    input->world_t_body = pose_status.target_p_source;
    input->world_R_body = pose_status.target_R_source;
    input->sensor = receiver_->sensor();

    queue_->push(input);
  }
}

}  // namespace hydra
