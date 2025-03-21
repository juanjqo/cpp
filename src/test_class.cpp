#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/solvers/DQ_GaussPrincipleSolver.h>


using namespace Eigen;
using namespace DQ_robotics;


int main(void)
{

    auto robot = std::static_pointer_cast<DQ_Dynamics>(
            std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::dynamics())
        );
    auto gp_solver = DQ_GaussPrincipleSolver(robot);

    std::cout<<"Gravity: "<<robot->get_gravity_acceleration()<<std::endl;
    std::cout<<"Bodies: "<<robot->get_number_of_bodies()<<std::endl;
    auto inertia_tensors = robot->get_inertia_tensors();
    for(auto& tensor: inertia_tensors)
    {
        std::cout<<tensor<<std::endl;
        std::cout<<" "<<std::endl;
    }
    VectorXd q = (VectorXd(7) << 1,2,3,4,5,6,7).finished();
    auto forces = gp_solver.compute_generalized_forces(q, q, q);
    std::cout<<"forces: "<<forces.transpose()<<std::endl;
    std::cout<<"Inertia M:"<<gp_solver.get_inertia_matrix()<<std::endl;
    std::cout<<"Coriolis V:"<<gp_solver.get_coriolis_vector().transpose()<<std::endl;
    std::cout<<"Grav forces:"<<gp_solver.get_gravitational_forces_vector().transpose()<<std::endl;
}
