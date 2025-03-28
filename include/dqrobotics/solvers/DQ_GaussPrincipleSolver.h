#pragma once
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_GaussPrinciple.h>
#include<dqrobotics/solvers/DQ_DynamicsSolver.h>

namespace DQ_robotics
{
class DQ_GaussPrincipleSolver: public DQ_DynamicsSolver, public DQ_GaussPrinciple
{
protected:
    void _compute_euler_lagrange(const VectorXd &q, const VectorXd &q_dot);
public:
    DQ_GaussPrincipleSolver(const std::shared_ptr<DQ_Dynamics>& robot);

    VectorXd compute_generalized_forces(const VectorXd& q,
                                        const VectorXd& q_dot,
                                        const VectorXd& q_dot_dot) override;

    MatrixXd get_inertia_matrix() const override;
    VectorXd get_coriolis_vector() const override;
    VectorXd get_gravitational_forces_vector() const override;
};
}


