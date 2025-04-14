#pragma once
#include "dqrobotics/robot_modeling/DQ_SerialManipulator.h"
#include<dqrobotics/solvers/DQ_DynamicsSolver.h>

namespace DQ_robotics
{
class DQ_NewtonEulerSolver: public DQ_DynamicsSolver
{
private:
    void _set_solver_parameters(const std::shared_ptr<DQ_Kinematics>& robot,
                                const std::vector<Matrix<double, 3, 3> > &inertia_tensors,
                                const std::vector<DQ> &center_of_masses,
                                const std::vector<double> &masses);
    bool solver_parameters_set_{false};

    std::vector<Matrix<double, 3,3>> inertia_tensors_;
    std::vector<DQ> center_of_masses_;
    std::vector<double> masses_;
    int n_links_;

    MatrixXd inertia_matrix_;
    VectorXd coriolis_vector_;
    VectorXd gravitational_forces_;


    std::vector<DQ> twists_;
    std::vector<DQ> twists_dot_;
    std::vector<DQ> wrenches_;
    std::vector<DQ> frames_x_ci_to_im1_;
    std::vector<DQ> frames_x_im1_to_i_;
    std::vector<DQ> frames_x_0_ci_;



    MatrixXd inertia_matrix_ne_;
    VectorXd coriolis_vector_ne_;
    VectorXd canonical_gravitational_forces_ne_;

    std::vector<DQ> _compute_wrenches_ne(const std::vector<DQ> &xs,
                                         const std::vector<DQ> &joint_twists,
                                         const std::vector<DQ> &joint_twists_dot,
                                         const DQ &gravity);

    void _forward_recursion(const std::vector<DQ> &xs, const std::vector<DQ> &joint_twists,
                            const std::vector<DQ> &joint_twists_dot);

    void _backward_recursion(const DQ &gravity);

    DQ _M3(const MatrixXd &inertia_tensor, const DQ &h);

protected:
    VectorXd gravitational_forces_ne_;
    int n_dim_space_;
    std::vector<DQ> xs_;

    VectorXd _compute_torques(const DQ& gravity, const VectorXd& q, const VectorXd& q_dot,
                              const VectorXd& q_dot_dot, const bool& fkm_flag = true);

    std::shared_ptr<DQ_SerialManipulator> serial_manipulator_;
    //std::shared_ptr<DQ_Dynamics> robot_;
public:
    DQ_NewtonEulerSolver();

    VectorXd compute_generalized_forces(const std::shared_ptr<DQ_Kinematics>& robot,
                                        const std::vector<Matrix<double, 3, 3> > &inertia_tensors,
                                        const std::vector<DQ> &center_of_masses,
                                        const std::vector<double> &masses,
                                        const DQ& gravity,
                                        const VectorXd& q,
                                        const VectorXd& q_dot,
                                        const VectorXd& q_dot_dot) override;

    MatrixXd compute_inertia_matrix(const std::shared_ptr<DQ_Kinematics>& robot,
                                    const std::vector<Matrix<double, 3, 3> > &inertia_tensors,
                                    const std::vector<DQ> &center_of_masses,
                                    const std::vector<double> &masses,
                                    const DQ& gravity,
                                    const VectorXd& q) override;

    VectorXd compute_coriolis_vector(const std::shared_ptr<DQ_Kinematics>& robot,
                                     const std::vector<Matrix<double, 3, 3> > &inertia_tensors,
                                     const std::vector<DQ> &center_of_masses,
                                     const std::vector<double> &masses,
                                     const DQ& gravity,
                                     const VectorXd& q,
                                     const VectorXd& q_dot) override;

    VectorXd compute_gravitational_forces_vector(const std::shared_ptr<DQ_Kinematics> &robot,
                                                 const std::vector<Matrix<double, 3, 3> > &inertia_tensors,
                                                 const std::vector<DQ> &center_of_masses,
                                                 const std::vector<double> &masses,
                                                 const DQ &gravity,
                                                 const VectorXd& q) override;
};
}
