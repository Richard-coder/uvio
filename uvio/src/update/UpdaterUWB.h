/*
 * Copyright (C) 2020 Alessandro Fornasier, Control of Networked Systems, University of Klagenfurt, Austria.
 *
 * All rights reserved.
 *
 * You can contact the author at <alessandro.fornasier@aau.at>
 */

#ifndef OV_MS_MSCKF_UPDATER_UWB_H_
#define OV_MS_MSCKF_UPDATER_UWB_H_

#include <Eigen/Eigen>
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include "utils/colors.h"

#include "UpdaterHelper.h"
#include "UpdaterOptions.h"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


namespace bw2_ms_msckf {

    class UpdaterUWB{

    public:

        UpdaterUWB(UpdaterOptions &_options_uwb) : _options_uwb(_options_uwb){
            // Initialize the chi squared test table with confidence level 0.95
            // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
            for (int i = 1; i < 500; i++) {
                boost::math::chi_squared chi_squared_dist(i);
                chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
            }
        }

        void update(std::shared_ptr<State> state, const std::shared_ptr<ov_core::UwbData>& message);

    protected:

        /// Options used during update
        UpdaterOptions _options_uwb;

        /// Chi squared 95th percentile table (lookup would be size of residual)
        std::map<int, double> chi_squared_table;
    };
}

#endif //OV_MS_MSCKF_UPDATER_UWB_H_
