/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef UVIO_TYPE_UWB_ANCHOR_H
#define UVIO_TYPE_UWB_ANCHOR_H

#include "types/Vec.h"

namespace uvio {

/**
 * @brief Derived Type class that implements an UWB anchor
 *
 * Contains the anchor position in global reference frame _p_AinG, constant and
 * distance bias of the measurement model.
 * The position is first, followed by constant bias, etc.
 */
class UWB_anchor : public ov_type::Type {

public:
  UWB_anchor(size_t anchor_id) : ov_type::Type(5) {

    // Set UWB anchor id
    _anchor_id = anchor_id;

    // Create all the sub-variables
    _p_AinG = std::shared_ptr<ov_type::Vec>(new ov_type::Vec(3));
    _const_bias = std::shared_ptr<ov_type::Vec>(new ov_type::Vec(1));
    _dist_bias = std::shared_ptr<ov_type::Vec>(new ov_type::Vec(1));

    // Set our default state value
    Eigen::VectorXd uwb_anchor0 = Eigen::VectorXd::Zero(5, 1);
    set_value_internal(uwb_anchor0);
    set_fej_internal(uwb_anchor0);
  }

  ~UWB_anchor() {}

  /**
   * @brief Sets id used to track location of variable in the filter covariance
   *
   * Note that we update the sub-variables also.
   *
   * @param new_id entry in filter covariance corresponding to this variable
   */
  inline void set_local_id(int new_id) override {
    _id = new_id;
    _p_AinG->set_local_id(new_id);
    _const_bias->set_local_id(_p_AinG->id() + ((new_id != -1) ? _p_AinG->size() : 0));
    _dist_bias->set_local_id(_const_bias->id() + ((new_id != -1) ? _const_bias->size() : 0));
  }

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  inline void set_value(const Eigen::MatrixXd &new_value) override { set_value_internal(new_value); }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  inline void set_fej(const Eigen::MatrixXd &new_value) override { set_fej_internal(new_value); }

  inline std::shared_ptr<ov_type::Type> clone() override {
    auto Clone = std::shared_ptr<ov_type::Type>(std::make_shared<UWB_anchor>(_anchor_id));
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  inline std::shared_ptr<ov_type::Type> check_if_subvariable(const std::shared_ptr<Type> check) override {
    if (check == _p_AinG) {
      return _p_AinG;
    } else if (check == _const_bias) {
      return _const_bias;
    } else if (check == _dist_bias) {
      return _dist_bias;
    }
    return nullptr;
  }

  /// Anchor id type access
  inline size_t anchor_id() { return _anchor_id; }

  /// Position type access
  inline std::shared_ptr<ov_type::Vec> p_AinG() { return _p_AinG; }

  /// Constant type access
  inline std::shared_ptr<ov_type::Vec> const_bias() { return _const_bias; }

  /// Distance bias access
  inline std::shared_ptr<ov_type::Vec> dist_bias() { return _dist_bias; }

protected:
  /// UWB anchor id
  size_t _anchor_id;

  /// Pose subvariable
  std::shared_ptr<ov_type::Vec> _p_AinG;

  /// Velocity subvariable
  std::shared_ptr<ov_type::Vec> _const_bias;

  /// Gyroscope bias subvariable
  std::shared_ptr<ov_type::Vec> _dist_bias;

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  inline void set_value_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 5);
    assert(new_value.cols() == 1);

    _p_AinG->set_value(new_value.block(0, 0, 3, 1));
    _const_bias->set_value(new_value.block(3, 0, 1, 1));
    _dist_bias->set_value(new_value.block(4, 0, 1, 1));

    _value = new_value;
  }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  inline void set_fej_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 5);
    assert(new_value.cols() == 1);

    _p_AinG->set_fej(new_value.block(0, 0, 3, 1));
    _const_bias->set_fej(new_value.block(3, 0, 1, 1));
    _dist_bias->set_fej(new_value.block(4, 0, 1, 1));

    _fej = new_value;
  }
};

} // namespace uvio

#endif // UVIO_TYPE_UWB_ANCHOR_H
