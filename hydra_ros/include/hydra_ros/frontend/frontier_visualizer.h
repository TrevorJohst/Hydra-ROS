#pragma once
#include <config_utilities/dynamic_config.h>
#include <hydra/frontend/frontier_extractor.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/utils/marker_group_pub.h>
#include <ianvs/node_handle.h>

namespace hydra {

class FrontierVisualizer : public FrontierExtractor::Sink {
 public:
  struct Config {
    std::string ns = "~/frontiers";
    double arrow_length = 0.5;
    visualizer::CategoricalColormap::Config label_colormap;
  } const config;

  explicit FrontierVisualizer(const Config& config);

  virtual ~FrontierVisualizer() = default;

  std::string printInfo() const override;

  void call(uint64_t timestamp_ns, const std::vector<Frontier>&) const override;

 private:
  void drawRayFronts(const std::vector<Frontier>& frontiers,
                     const std::string& ns) const;

 protected:
  ianvs::NodeHandle nh_;
  MarkerGroupPub pubs_;
  const visualizer::CategoricalColormap label_colormap_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<FrontierExtractor::Sink,
                                     FrontierVisualizer,
                                     Config>("FrontierVisualizer");
};

void declare_config(FrontierVisualizer::Config& config);

}  // namespace hydra
