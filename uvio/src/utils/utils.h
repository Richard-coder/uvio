/*
 * UVIO: Ultra Wide-Band aided Visual-Inertial Odometry
 * Copyright (C) 2020-2022 Alessandro Fornasier
 * Copyright (C) 2018-2022 UVIO Contributors
 *
 * Control of Networked Systems, University of Klagenfurt, Austria (alessandro.fornasier@aau.at)
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

#ifndef UVIO_UTILS_H
#define UVIO_UTILS_H

#include <string>
#include <sstream>
#include <cctype>

namespace uvio
{

/**
 * @brief containsChar: Check if a string contains character.
 * True is a string contians characters, False if a string is only made of numbers
 *
 * @param[in] string
 * @return bool
 */
inline bool containsChar(const std::string& str)
{
    for (char const &c : str) {
        if (std::isdigit(c) == 0) return true;
    }
    return false;
}

}

#endif // UVIO_UTILS_H
