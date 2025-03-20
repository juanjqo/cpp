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
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/robot_modeling/DQ_Kinetics.h>

namespace DQ_robotics
{
class DQ_Dynamics: public DQ_Kinematics, public DQ_Kinetics
{
protected:
    DQ gravity_acceleration_ = -9.81*k_;
    DQ_Dynamics();
public:
    virtual ~DQ_Dynamics() = default;
    void set_gravity_acceleration(const DQ& gravity_acceleration);
    DQ get_gravity_acceleration() const;
};
}


