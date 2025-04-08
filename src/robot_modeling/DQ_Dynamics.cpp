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

#include <dqrobotics/robot_modeling/DQ_Dynamics.h>

namespace DQ_robotics
{
DQ_Dynamics::DQ_Dynamics() {}

void DQ_Dynamics::set_gravity_acceleration(const DQ &gravity_acceleration)
{
    if (!is_pure_quaternion(gravity_acceleration))
    {
        throw std::runtime_error(std::string("The gravity must be a pure quaternion!"));
    }else{
        gravity_acceleration_ = gravity_acceleration;
    }
}

DQ DQ_Dynamics::get_gravity_acceleration() const
{
    return gravity_acceleration_;
}

void DQ_Dynamics::dynamic_solver(const std::shared_ptr<DQ_DynamicsSolver> &solver)
{
    dynamic_solver_ = solver;
}


VectorXd DQ_Dynamics::compute_generalized_forces(const VectorXd &q, const VectorXd &q_dot, const VectorXd &q_dot_dot)
{
    if (!dynamic_solver_)
    {
        throw std::runtime_error("Undefined solver!");
    }
    return VectorXd::Zero(1);
}




}
