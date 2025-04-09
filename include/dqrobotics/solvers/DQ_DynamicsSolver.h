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
#include "dqrobotics/robot_modeling/DQ_Kinematics.h"
#include <dqrobotics/DQ.h>
//#include<dqrobotics/robot_modeling/DQ_Dynamics.h>
#include<memory>
#include<vector>

using namespace Eigen;

namespace DQ_robotics
{
class DQ_DynamicsSolver
{
protected:
   //std::shared_ptr<DQ_Kinematics> robot_;
   VectorXd q_;
   VectorXd dq_;
   VectorXd ddq_;
   //DQ_DynamicsSolver(const std::shared_ptr<DQ_Dynamics>& robot);
   DQ_DynamicsSolver();
public:
   virtual VectorXd compute_generalized_forces(const VectorXd& q,
                                               const VectorXd& q_dot,
                                               const VectorXd& q_dot_dot) = 0;

   virtual MatrixXd compute_inertia_matrix(const VectorXd& q) = 0;

   virtual VectorXd compute_coriolis_vector(const VectorXd& q,
                                            const VectorXd& q_dot) = 0;

   virtual VectorXd compute_gravitational_forces_vector(const VectorXd& q) = 0;

};
}


