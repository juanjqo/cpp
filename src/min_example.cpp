#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/solvers/DQ_GaussPrincipleSolver.h>
#include <dqrobotics/solvers/DQ_NewtonEulerSolver.h>


using namespace Eigen;
using namespace DQ_robotics;

int main(void)
{
    auto robot = std::make_shared<DQ_SerialManipulatorMDH>
        (FrankaEmikaPandaRobot::dynamics());
    //robot->set_gravity_acceleration(DQ{0});

    const auto q      = (VectorXd(7) << 1,2,3,4,0,0,0).finished();
    const auto q_dot  = (VectorXd(7) << 0,0,0,5,6,7,8).finished();
    const auto q_ddot = (VectorXd(7) << -1,0,-4,0,0,0,0).finished();

    //Gauss Principle of Least Constraint solver

    robot->dynamic_solver(std::make_shared<DQ_GaussPrincipleSolver>());
    auto forces = robot->compute_generalized_forces(q, q_dot, q_ddot);
    std::cout<<forces.transpose()<<std::endl;

    auto M = robot->compute_inertia_matrix(q);
    auto c = robot->compute_coriolis_vector(q, q_dot);
    auto g = robot->compute_gravitational_forces_vector(q);

    auto robot2 = std::make_shared<DQ_SerialManipulatorMDH>
        (FrankaEmikaPandaRobot::dynamics());
    //robot2->set_gravity_acceleration(DQ{0});

    robot2->dynamic_solver(std::make_shared<DQ_NewtonEulerSolver>());
    //std::cout<<"error: "<<(forces-robot2->compute_generalized_forces(q, q_dot, q_ddot)).transpose()<<std::endl;
    std::cout<<(robot2->compute_generalized_forces(q, q_dot, q_ddot)).transpose()<<std::endl;
    // -2.02579  -22.8425   2.63973   6.05451   1.74369 -0.171619 -0.724505
}



