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

#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <Eigen/StdVector>

namespace DQ_robotics
{

///****************************************************************************************
///                        PRIVATE FUNCTIONS
/// ***************************************************************************************
/**
 * @brief _get_mdh_matrix returns a matrix related to the modified D-H parameters of the
 *                        Franka Emika Panda robot, which is
 *                        defined as
 *
 *                        Matrix<double, 5, 6> raw_franka_mdh;
 *                        raw_franka_mdh << theta,
 *                                              d,
 *                                              a,
 *                                           alpha,
 *                                            type_of_joints;
 * Source: https://frankaemika.github.io/docs/control_parameters.html
 *
 * @return MatrixXd raw_franka_mdh a matrix related to the modified D-H parameters
 */
MatrixXd _get_mdh_matrix()
{
    const double pi2 = pi/2.0;
    Matrix<double,5,7> raw_franka_mdh(5,7);
    raw_franka_mdh <<  0,    0,       0,         0,         0,      0,      0,
                     0.333,  0, 3.16e-1,         0,   3.84e-1,      0,      0,
                      0,     0,       0,   8.25e-2,  -8.25e-2,      0, 8.8e-2,
                      0,  -pi2,     pi2,       pi2,      -pi2,    pi2,    pi2,
                      0,     0,       0,         0,         0,      0,      0;

    return raw_franka_mdh;
}


/**
 * @brief _get_offset_base returns the base offset of the Franka Emika Panda robot.
 * @return a unit dual quatenion representing the base offset of the robot.
 */
DQ _get_offset_base()
{
    return 1 + E_ * 0.5 * DQ(0, 0.0413, 0, 0);
}

/**
 * @brief _get_offset_flange returns the end-effector offset of the Franka Emika Panda robot,
 *                           which is called "Flange" by the manufacturer.
 *                           This offset does not correspond to the end-effector tool but is part
 *                           of the modeling based on Craig's convention.
 * Source: https://frankaemika.github.io/docs/control_parameters.html
 * @return
 */
DQ _get_offset_flange()
{
    return 1+E_*0.5*k_*1.07e-1;
}

/**
 * @brief _get_q_limits returns the joint limits of the Franka Emika Panda robot
 *                      as suggested by the manufacturer.
 *
 * source: https://frankaemika.github.io/docs/control_parameters.html
 *
 * @return a std::make_tuple(q_min_, q_max_) containing the joint limits.
 */
std::tuple<const VectorXd, const VectorXd> _get_q_limits()
{
    const VectorXd q_max_ = ((VectorXd(7) <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished());
    const VectorXd q_min_ = ((VectorXd(7) << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished());
    return std::make_tuple(q_min_, q_max_);
}


/**
 * @brief _get_q_dot_limits returns the joint velocity limits of the Franka Emika Panda robot
 *                          as suggested by the manufacturer.
 *
 * source: https://frankaemika.github.io/docs/control_parameters.html
 *
 * @return a std::make_tuple(q_min_dot_, q_max_dot_) containing the joint velocity limits.
 */
std::tuple<const VectorXd, const VectorXd> _get_q_dot_limits()
{
    const VectorXd q_min_dot_ = ((VectorXd(7) << -2, -1, -1.5, -1.25, -3, -1.5, -3).finished());
    const VectorXd q_max_dot_ = ((VectorXd(7) <<  2,  1,  1.5,  1.25,  3,  1.5,  3).finished());
    return std::make_tuple(q_min_dot_, q_max_dot_);
}




///****************************************************************************************
///                        PUBLIC FUNCTIONS
/// ***************************************************************************************


/**
 * @brief FrankaEmikaPandaRobot::kinematics returns an object of the class DQ_SerialManipulatorMDH()
 *                                          that contain the kinematic model of the Franka Emika Panda robot.
 *           Example of use:
 *
 *           #include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
 *           DQ_SerialManipulatorMDH franka = FrankaEmikaPandaRobot::kinematics();
 *           DQ x = franka.fkm(VectorXd::Zero(franka.get_dim_configuration_space()));
 *
 * @return A DQ_SerialManipulatorMDH() object of the desire robot
 */
DQ_SerialManipulatorMDH FrankaEmikaPandaRobot::kinematics()
{
    DQ_SerialManipulatorMDH franka(_get_mdh_matrix());
    franka.set_base_frame(_get_offset_base());
    franka.set_reference_frame(_get_offset_base());
    franka.set_effector(_get_offset_flange());
    VectorXd q_min;
    VectorXd q_max;
    VectorXd q_dot_min;
    VectorXd q_dot_max;
    std::tie(q_min, q_max) = _get_q_limits();
    std::tie(q_dot_min, q_dot_max) = _get_q_dot_limits();
    franka.set_lower_q_limit(q_min);
    franka.set_upper_q_limit(q_max);
    franka.set_lower_q_dot_limit(q_dot_min);
    franka.set_upper_q_dot_limit(q_dot_max);
    return franka;
}

DQ_SerialManipulatorMDH FrankaEmikaPandaRobot::dynamics()
{
    std::vector<Matrix<double, 3,3>> inertia_tensors(7);
    inertia_tensors[0] =  (MatrixXd(3,3) <<
                              0.703461, -2.24141e-07, -1.27891e-06,
                             -2.24141e-07, 0.7072, -3.60235e-07,
                             -1.27891e-06, -3.60235e-07, 0.00862065).finished();

    inertia_tensors[1] =  (MatrixXd(3,3) <<
                              0.00323353, -1.3719e-06, -1.50498e-07,
                          -1.3719e-06, 0.0284347, 3.99273e-07,
                          -1.50498e-07, 3.99273e-07, 0.0314941).finished();

    inertia_tensors[2] =  (MatrixXd(3,3) <<
                              0.0564926, -0.00824819, -0.00549347,
                          -0.00824819, 0.0528789, -0.00437187,
                          -0.00549347, -0.00437187, 0.0182495).finished();

    inertia_tensors[3] =  (MatrixXd(3,3) <<
                              0.0676859, 0.0277108, 0.00390868,
                          0.0277108, 0.0323926, -0.00165116,
                          0.00390868, -0.00165116, 0.0775836).finished();

    inertia_tensors[4] =  (MatrixXd(3,3) <<
                              0.0394288, -0.00151486, -0.00459741,
                          -0.00151486, 0.0314597, 0.00216469,
                          -0.00459741, 0.00216469, 0.0108687).finished();

    inertia_tensors[5] =  (MatrixXd(3,3) <<
                              0.00274918, 0.00166276, 0.00140341,
                          0.00166276, 0.0114962, -0.000256865,
                          0.00140341, -0.000256865, 0.0105974).finished();

    inertia_tensors[6] =  (MatrixXd(3,3) <<
                              0.0153198, -0.000395258, -0.00167357,
                          -0.000395258, 0.0128986, -0.000549074,
                          -0.00167357, -0.000549074, 0.00491012).finished();

    std::vector<Vector3d> center_of_masses(7);
    center_of_masses[0] = (Vector3d() <<0.00359005, 0.00207575, -2.93732e-07).finished();
    center_of_masses[1] = (Vector3d() <<-0.00342111, -0.0287195, 0.00349458).finished();
    center_of_masses[2] = (Vector3d() <<0.0272627, 0.0392118, -0.0664985).finished();
    center_of_masses[3] = (Vector3d() <<-0.0533997, 0.104436, 0.0275176).finished();
    center_of_masses[4] = (Vector3d() <<-0.0123109, 0.0410224, -0.0384166).finished();
    center_of_masses[5] = (Vector3d() <<0.0599899, -0.0141342, -0.0104707).finished();
    center_of_masses[6] = (Vector3d() <<0.0103629, -0.00420036, -0.0453707).finished();

    std::vector<double> masses = {4.97068, 0.646926,   3.2286,  3.58789,  1.22595,  1.66656, 0.735522};
    //DQ_SerialManipulatorMDH robot =  FrankaEmikaPandaMDHRobot::kinematics();
    //auto arm = std::make_shared<DQ_SerialManipulatorMDH>(robot);
    //std::static_pointer_cast<DQ_Kinematics>(arm)

    DQ_SerialManipulatorMDH franka(_get_mdh_matrix(), inertia_tensors, center_of_masses, masses);
    franka.set_base_frame(_get_offset_base());
    franka.set_reference_frame(_get_offset_base());
    franka.set_effector(_get_offset_flange());
    return franka;
}

}

