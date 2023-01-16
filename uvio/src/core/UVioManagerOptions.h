#ifndef UVIOMANAGEROPTIONS_H
#define UVIOMANAGEROPTIONS_H

#include "core/VioManagerOptions.h"
#include "state/UVioStateOptions.h"
#include "update/UVioUpdaterOptions.h"
#include "utils/uvio_sensor_data.h"

namespace uvio {

/**
 * @brief Struct which stores uvio specific options
 */
struct UVioManagerOptions : ov_msckf::VioManagerOptions {
  /// UVIO specific options
  UVioStateOptions uvio_state_options;

  /// uwb extrinsics (p_IinU).
  Eigen::Vector3d uwb_extrinsics;

  /// uwb anchors references (id, p_AinG, const_bias, dist_bias, Cov).
  std::vector<AnchorData> uwb_anchor_extrinsics;

  /// UWB options
  UVioUpdaterOptions uwb_options;
};

} // namespace uvio

#endif // UVIOMANAGEROPTIONS_H
