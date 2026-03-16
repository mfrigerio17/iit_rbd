/* CPYHDR { */
/*
 * SPDX-FileCopyrightText: © 2026 Marco Frigerio
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * This file is part of the 'iit-rbd' library.
 */
/* } CPYHDR */

#ifndef IIT_RBD_DATA_MAP_H_
#define IIT_RBD_DATA_MAP_H_

#include <array>

namespace iit {
namespace rbd {

/**
 * A simple container to associate a value to each link/joint of a robot.
 * This class is used by RobCoGen-erated code.
 */
template<typename T, std::size_t Size, typename ItemID>
struct DataMap : public std::array<T, Size>
{
    using Base = std::array<T, Size>;
    using Base::begin;
    using Base::end;

    DataMap(const T& defaultValue) {
        assigndata(defaultValue);
    }
    DataMap& operator=(const T& rhs) {
        assigndata(rhs);
        return *this;
    }

    T& operator[](ItemID which) {
        return Base::operator[](which);
    }
    const T& operator[](ItemID which) const {
        return Base::operator[](which);
    }

private:
    void assigndata(const T& value) {
        std::fill(begin(), end(), value);
    }
};



}
}
#endif
