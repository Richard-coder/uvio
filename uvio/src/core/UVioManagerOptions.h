#ifndef UVIOMANAGEROPTIONS_H
#define UVIOMANAGEROPTIONS_H

#include <core/VioManagerOptions.h>
#include <state/UVioStateOptions.h>

namespace uvio {

/**
 * @brief Struct which stores uvio specific options
 */
struct UVioManagerOptions : ov_msckf::VioManagerOptions {
  /// UVIO specific options
  UVioStateOptions uvio_state_options;
};

} // namespace uvio

#endif // UVIOMANAGEROPTIONS_H
