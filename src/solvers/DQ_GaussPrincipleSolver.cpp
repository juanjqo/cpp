#include <dqrobotics/solvers/DQ_GaussPrincipleSolver.h>

namespace DQ_robotics
{

DQ_GaussPrincipleSolver::DQ_GaussPrincipleSolver(std::shared_ptr<DQ_Dynamics> &robot):
    DQ_DynamicsSolver(robot)
{

    DQ_GaussPrinciple::inertia_tensors_ = robot_->get_inertia_tensors();
    DQ_GaussPrinciple::center_of_masses_ = robot_->get_center_of_masses();
    DQ_GaussPrinciple::masses_ = robot_->get_masses();
    DQ_GaussPrinciple::n_links_ = masses_.size();
    MatrixXd zeros_8xn = MatrixXd::Zero(8, n_links_);
    for(int i=0; i<n_links_;i++)
    {
        auto I = inertia_tensors_[i];
        Psi_.push_back((MatrixXd(8,8) << 0,       0,        0,       0,   0,         0,          0,          0,
                        0,  I(0,0),   I(0,1),   I(0,2),  0,         0,          0,          0,
                        0,  I(1,0),   I(1,1),   I(1,2),  0,         0,          0,          0,
                        0,  I(2,0),   I(2,1),   I(2,2),  0,         0,          0,          0,
                        0,       0,        0,        0,  0,         0,          0,          0,
                        0,       0,        0,        0,  0, masses_[i],          0,          0,
                        0,       0,        0,        0,  0,         0,  masses_[i],          0,
                        0,       0,        0,        0,  0,         0,          0,   masses_[i]).finished());
    }
}

VectorXd DQ_GaussPrincipleSolver::compute_generalized_forces(const VectorXd &q, const VectorXd &q_dot, const VectorXd &q_dot_dot)
{
    _compute_euler_lagrange(q, q_dot);
    return inertia_matrix_gp_*q_dot_dot + coriolis_vector_gp_ + gravitational_forces_gp_;
}

void DQ_GaussPrincipleSolver::_compute_euler_lagrange(const VectorXd &q, const VectorXd &q_dot)
{
    int n_links = robot_->get_dim_configuration_space();
    MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_links);
    std::vector<DQ> xs;
    std::vector<MatrixXd> Js;
    std::vector<MatrixXd> Js_dot;

    for(int i=0; i<n_links;i++)
    {
        xs.push_back(DQ(0));
        Js.push_back(zeros_8_nlinks);
        Js_dot.push_back(zeros_8_nlinks);

        xs[i] = robot_->fkm(q,i);
        Js[i] = robot_->pose_jacobian(q,i);
        Js_dot[i] = robot_->pose_jacobian_derivative(q, q_dot, i);
    }
    DQ_GaussPrinciple::_compute_euler_lagrange_gp(xs, Js, Js_dot, q_dot, robot_->get_gravity_acceleration());

}

MatrixXd DQ_GaussPrincipleSolver::get_inertia_matrix() const
{
    return inertia_matrix_gp_;
}

VectorXd DQ_GaussPrincipleSolver::get_coriolis_vector() const
{
    return coriolis_vector_gp_;
}

VectorXd DQ_GaussPrincipleSolver::get_gravitational_forces_vector() const
{
    return gravitational_forces_gp_;
}



}
