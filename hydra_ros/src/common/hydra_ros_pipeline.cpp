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
#include "hydra_ros/common/hydra_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/backend/backend_module.h>
#include <hydra/common/hydra_config.h>
#include <hydra/frontend/frontend_module.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <hydra/reconstruction/reconstruction_module.h>

#include <memory>

#include "hydra_ros/backend/ros_backend_publisher.h"
#include "hydra_ros/common/input_module.h"
#include "hydra_ros/frontend/ros_frontend_publisher.h"

namespace hydra {

void declare_config(HydraRosConfig& conf) {
  using namespace config;
  name("HydraRosConfig");
  field(conf.enable_frontend_output, "enable_frontend_output");
}

HydraRosPipeline::HydraRosPipeline(const ros::NodeHandle& nh, int robot_id)
    : HydraPipeline(config::fromRos<PipelineConfig>(nh), robot_id),
      config_(config::checkValid(config::fromRos<HydraRosConfig>(nh))),
      nh_(nh) {}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::init() {
  const auto& pipeline_config = HydraConfig::instance().getConfig();
  initFrontend();
  initBackend();
  initReconstruction();
  if (pipeline_config.enable_lcd) {
    initLCD();
  }

  const auto reconstruction = getModule<ReconstructionModule>("reconstruction");
  CHECK(reconstruction);
  input_module_ = config::createFromROS<InputModule>(ros::NodeHandle(nh_, "input"),
                                                     reconstruction->queue());
}

void HydraRosPipeline::start() {
  VLOG(config_verbosity_) << std::endl << getModuleInfo("input", input_module_.get());

  showModules();

  input_module_->start();
  for (auto&& [name, module] : modules_) {
    if (!module) {
      LOG(FATAL) << "Found unitialized module: " << name;
    }

    module->start();
  }

  modules_["input"] = input_module_;
}

void HydraRosPipeline::initFrontend() {
  ros::NodeHandle fnh(nh_, "frontend");
  const auto logs = HydraConfig::instance().getLogs();
  FrontendModule::Ptr frontend =
      config::createFromROS<FrontendModule>(fnh, frontend_dsg_, shared_state_, logs);
  if (config_.enable_frontend_output) {
    CHECK(frontend) << "Frontend module required!";
    frontend->addSink(std::make_shared<RosFrontendPublisher>(fnh));
  }

  modules_["frontend"] = frontend;
}

void HydraRosPipeline::initBackend() {
  ros::NodeHandle bnh(nh_, "backend");
  const auto logs = HydraConfig::instance().getLogs();
  BackendModule::Ptr backend =
      config::createFromROS<BackendModule>(bnh, backend_dsg_, shared_state_, logs);
  CHECK(backend) << "Failed to construct backend!";
  backend->addSink(std::make_shared<RosBackendPublisher>(bnh));
  modules_["backend"] = backend;
}

void HydraRosPipeline::initReconstruction() {
  const auto frontend = getModule<FrontendModule>("frontend");
  if (!frontend) {
    LOG(ERROR) << "Invalid frontend module: disabling reconstruction";
    return;
  }

  ros::NodeHandle rnh(nh_, "reconstruction");
  modules_["reconstruction"] =
      config::createFromROS<ReconstructionModule>(rnh, frontend->getQueue());
}

void HydraRosPipeline::initLCD() {
  shared_state_->lcd_queue.reset(new InputQueue<LcdInput::Ptr>());
  const ros::NodeHandle lcd_nh(nh_, "loop_closure");
  modules_["lcd"] = config::createFromROS<LoopClosureModule>(lcd_nh, shared_state_);
}

}  // namespace hydra
