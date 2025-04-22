#pragma once
#include "dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h"
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDenso.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include<dqrobotics/robot_modeling/DQ_SerialWholeBody.h>
#include<dqrobotics/robot_modeling/DQ_GaussPrinciple.h>
#include<dqrobotics/solvers/DQ_DynamicsSolver.h>

namespace DQ_robotics
{
class DQ_GaussPrincipleSolver: public DQ_DynamicsSolver
{
private:
    void _set_solver_parameters(const std::shared_ptr<DQ_Kinematics>& robot,
                                const std::vector<Matrix<double, 3, 3> > &inertia_tensors,
                                const std::vector<DQ> &center_of_masses,
                                const std::vector<double> &masses);

    bool solver_parameters_set_{false};

protected:


    std::vector<Matrix<double, 8,8>>  Psi_;
    std::vector<MatrixXd>  J_;
    std::vector<MatrixXd>  J_dot_;
    std::vector<MatrixXd>  Jecom_;
    std::vector<MatrixXd>  Jecom_dot_;
    std::vector<MatrixXd>  Z_;
    std::vector<DQ>  xcoms_;
    std::vector<DQ>  xs_;
    int n_links_;
    int n_dim_space_;

    MatrixXd inertia_matrix_gp_;
    VectorXd coriolis_vector_gp_;
    VectorXd gravitational_forces_gp_;

    DQ default_gravity_{-9.81*k_};
    DQ current_gravity_;

    std::vector<Matrix<double, 3,3>> inertia_tensors_;
    std::vector<DQ> center_of_masses_;
    std::vector<double> masses_;



enum class ROBOT_TYPE
    {
        SERIAL_MANIPULATOR, SERIAL_WHOLE_BODY, HOLONOMIC_BASE
    };
    ROBOT_TYPE robot_type_;

    ROBOT_TYPE _get_robot_type(const std::shared_ptr<DQ_Kinematics>& kinematics);

    bool _update_configuration(const VectorXd &q);
    bool _update_configuration_velocities(const VectorXd &q_dot);



    DQ _get_fkm(const std::shared_ptr<DQ_Kinematics>& kinematics,
                const VectorXd &q,
                const int &to_ith_link);

    MatrixXd _get_pose_jacobian(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                const VectorXd &q,
                                const int &to_ith_link);

    MatrixXd _get_pose_jacobian_derivative(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                           const VectorXd &q,
                                           const VectorXd &q_dot,
                                           const int &to_ith_link);


    void _compute_first_order_components(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                         const VectorXd &q, const DQ& gravity);
    void _compute_second_order_components(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                          const VectorXd &q, const VectorXd &q_dot);




    static MatrixXd twist_jacobian(const MatrixXd &pose_jacobian, const DQ &pose);

    static MatrixXd twist_jacobian_derivative(const MatrixXd &pose_jacobian,
                                              const MatrixXd &pose_jacobian_derivative,
                                              const DQ &pose,
                                              const VectorXd &q_dot);


    std::shared_ptr<DQ_SerialManipulator> serial_manipulator_;
    std::shared_ptr<DQ_SerialWholeBody> serial_whole_body_;
    std::shared_ptr<DQ_HolonomicBase> holonomic_base_;
    std::shared_ptr<DQ_DifferentialDriveRobot> differential_base_;

public:
    //DQ_GaussPrincipleSolver(const std::shared_ptr<DQ_Dynamics>& robot);
    //virtual ~DQ_GaussPrincipleSolver();
    DQ_GaussPrincipleSolver();

    VectorXd compute_generalized_forces(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                        const std::shared_ptr<DQ_Kinetics>& kinetics,
                                        const DQ& gravity,
                                        const VectorXd& q,
                                        const VectorXd& q_dot,
                                        const VectorXd& q_dot_dot) override;

    MatrixXd compute_inertia_matrix(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                    const std::shared_ptr<DQ_Kinetics>& kinetics,
                                    const VectorXd& q) override;

    VectorXd compute_coriolis_vector(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                     const std::shared_ptr<DQ_Kinetics>& kinetics,
                                     const VectorXd& q,
                                     const VectorXd& q_dot) override;

    VectorXd compute_gravitational_forces_vector(const std::shared_ptr<DQ_Kinematics>& kinematics,
                                                 const std::shared_ptr<DQ_Kinetics>& kinetics,
                                                 const DQ& gravity,
                                                 const VectorXd& q) override;

};
}


