/**
(C) Copyright 2011-2025 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
*/

#pragma once

#include<dqrobotics/DQ.h>
#include<memory>

namespace DQ_robotics
{
class DQ_Kinetics
{
private:
    template<typename T>
    void _check_parameters(const T& parameters, const std::string& error_msg) const {
        if (parameters.size() == 0)
            throw std::runtime_error(error_msg);
    }
    void _check_dim_inertial_parameters();

    std::vector<Matrix<double, 3,3>> inertia_tensors_;
    std::vector<Vector3d> center_of_masses_;
    std::vector<double> masses_;
    int number_of_bodies_;

protected:


    DQ_Kinetics();

public:
    virtual ~DQ_Kinetics() = default;
    std::vector<Matrix<double, 3,3>>  get_inertia_tensors() const;
    std::vector<Vector3d>             get_center_of_masses() const;
    std::vector<double>               get_masses() const;
    int                               get_number_of_bodies() const;

    //void set_inertia_tensors(const std::vector<Matrix<double,3,3>>& inertia_tensors);
    //void set_center_of_masses(const std::vector<Vector3d>& center_of_masses);
    //void set_masses(const std::vector<double>& masses);
    void set_inertial_parameters(const std::vector<Matrix<double,3,3>>& inertia_tensors,
                                 const std::vector<Vector3d>& center_of_masses,
                                 const std::vector<double>& masses
                                 );


};
}
