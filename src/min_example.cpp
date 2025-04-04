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

    auto q      = (VectorXd(7) << 1,2,3,4,0,0,0).finished();
    auto q_dot  = (VectorXd(7) << 0,0,0,5,6,7,8).finished();
    auto q_ddot = (VectorXd(7) << -1,0,-4,0,0,0,0).finished();

    // Gauss Principle of Least Constraint solver      Newton-Euler solver
    auto rd_solver = DQ_GaussPrincipleSolver(robot); //DQ_NewtonEulerSolver(robot);
    auto forces = rd_solver.compute_generalized_forces(q, q_dot, q_ddot);
    auto M = rd_solver.compute_inertia_matrix(q);
    auto c = rd_solver.compute_coriolis_vector(q, q_dot);
    auto g = rd_solver.compute_gravitational_forces_vector(q);
}



