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

#include <dqrobotics/robot_modeling/DQ_Kinetics.h>
namespace DQ_robotics
{
DQ_Kinetics::DQ_Kinetics(){

}

std::vector<Matrix<double, 3, 3> > DQ_Kinetics::get_inertia_tensors() const
{
    _check_parameters(inertia_tensors_, "Bad call in get_inertia_tensors(). The inertia tensors are not defined.");
    return inertia_tensors_;
}

std::vector<Vector3d> DQ_Kinetics::get_center_of_masses() const
{
    _check_parameters(inertia_tensors_, "Bad call in get_center_of_masses(). The center of masses are not defined.");
    return center_of_masses_;
}

std::vector<double> DQ_Kinetics::get_masses() const
{
    _check_parameters(inertia_tensors_, "Bad call in get_masses(). The masses are not defined.");
    return masses_;
}

int DQ_Kinetics::get_number_of_bodies() const
{
    return number_of_bodies_;
}

void DQ_Kinetics::set_inertia_tensors(const std::vector<Matrix<double, 3, 3> > &inertia_tensors)
{
    inertia_tensors_ = inertia_tensors;
}

void DQ_Kinetics::set_center_of_masses(const std::vector<Vector3d> &center_of_masses)
{
    center_of_masses_ = center_of_masses;
}

void DQ_Kinetics::set_masses(const std::vector<double> &masses)
{
    masses_ = masses;
}

void DQ_Kinetics::_check_dim_inertial_parameters()
{
    _check_parameters(masses_, "The masses are not defined.");
    _check_parameters(center_of_masses_, "The center of masses are not defined.");
    _check_parameters(inertia_tensors_, "The inertia tensors are not defined.");

    std::size_t s_masses = masses_.size();
    std::size_t s_coms   = center_of_masses_.size();
    std::size_t s_inertias = inertia_tensors_.size();

    if ((s_masses == s_coms) && (s_coms == s_inertias))
        number_of_bodies_ = s_masses;
    else
        throw std::runtime_error(std::string("The inertial parameters have incompatible sizes!"));
}

}

