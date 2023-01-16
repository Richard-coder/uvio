/*
 * Copyright (C) 2020 Alessandro Fornasier, Control of Networked Systems, University of Klagenfurt, Austria.
 *
 * All rights reserved.
 *
 * You can contact the author at <alessandro.fornasier@aau.at>
 */

#ifndef UVIO_UPDATER_UWB_H_
#define UVIO_UPDATER_UWB_H_

#include <Eigen/Eigen>
#include "state/UVioState.h"
#include "utils/quat_ops.h"
#include "utils/colors.h"

#include "update/UVioUpdaterHelper.h"
#include "update/UVioUpdaterOptions.h"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


namespace uvio {

class UpdaterUWB{

public:

  UpdaterUWB(UVioUpdaterOptions &options_) : _options(options_) {
    // Initialize the chi squared test table with confidence level 0.95
    // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
    for (int i = 1; i < 500; i++) {
      boost::math::chi_squared chi_squared_dist(i);
      _chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
    }
  }

  void update(std::shared_ptr<UVioState> state, const std::shared_ptr<UwbData>& message);

protected:

  /// Options used during update
  UVioUpdaterOptions _options;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> _chi_squared_table;
};
}

#endif //UVIO_UPDATER_UWB_H_
