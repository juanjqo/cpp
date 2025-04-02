#pragma once
#include<dqrobotics/DQ.h>

namespace DQ_robotics
{
class DQ_GaussPrinciple
{
protected:
    std::vector<Matrix<double, 3,3>> inertia_tensors_;
    std::vector<Vector3d> center_of_masses_;
    std::vector<double> masses_;

    std::vector<Matrix<double, 8,8>>  Psi_;
    std::vector<MatrixXd>  J_;
    std::vector<MatrixXd>  J_dot_;
    std::vector<MatrixXd>  Jecom_;
    std::vector<MatrixXd>  Jecom_dot_;
    std::vector<DQ>  xcoms_;
    std::vector<DQ>  xs_;
    int n_links_;

    MatrixXd inertia_matrix_gp_;
    VectorXd coriolis_vector_gp_;
    VectorXd gravitational_forces_gp_;

    void    _compute_euler_lagrange_gp(const std::vector<DQ> &xs,
                                    const std::vector<MatrixXd> &Js,
                                    const std::vector<MatrixXd> &Js_dot,
                                    const VectorXd &q_dot,
                                    const DQ &gravity);



    static MatrixXd twist_jacobian(const MatrixXd &pose_jacobian, const DQ &pose);
    static MatrixXd twist_jacobian_derivative(const MatrixXd &pose_jacobian,
                                              const MatrixXd &pose_jacobian_derivative,
                                              const DQ &pose,
                                              const VectorXd &q_dot);

    static MatrixXd _compute_inertia_matrix(const std::vector<MatrixXd> &Jecom, std::vector<Matrix<double, 8,8>> &Psi);

    static VectorXd _compute_coriolis_vector(const std::vector<MatrixXd> &Jecom, std::vector<Matrix<double, 8,8>> &Psi,
                                             const std::vector<MatrixXd> &Jecom_dot, const VectorXd& q_dot);

    static VectorXd _compute_gravitational_forces(const std::vector<MatrixXd> &Jecom, std::vector<Matrix<double, 8,8>> &Psi,
                                                  const std::vector<DQ> &xcoms, const DQ &gravity);
};
}


