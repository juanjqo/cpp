#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/solvers/DQ_GaussPrincipleSolver.h>
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>


using namespace Eigen;
using namespace DQ_robotics;


int main(void)
{

    auto robot = std::static_pointer_cast<DQ_Dynamics>(
      std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::dynamics())
        );

    auto robot2 = std::static_pointer_cast<DQ_Dynamics>(
        std::make_shared<DQ_HolonomicBase>()
        );

    auto robot3 = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::dynamics());

    auto gp_solver = DQ_GaussPrincipleSolver(robot);

    std::cout<<"Gravity: "<<robot->get_gravity_acceleration()<<std::endl;
    std::cout<<"Bodies: "<<robot->get_number_of_bodies()<<std::endl;
    auto inertia_tensors = robot->get_inertia_tensors();
    /*
    for(auto& tensor: inertia_tensors)
    {
        std::cout<<tensor<<std::endl;
        std::cout<<" "<<std::endl;
    }
    */
    VectorXd q = (VectorXd(7) << 1,2,3,4,5,6,7).finished();


    auto initial_time_ = std::chrono::steady_clock::now();


    auto forces = gp_solver.compute_generalized_forces(q, q, q);


    const auto end{std::chrono::steady_clock::now()};
    auto elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
    std::cout<<"elapsed time: "<<elapsed_seconds_.count()*1e3<<" ms"<<std::endl;


    VectorXd v1 = (VectorXd(3)<<1,2,3+DQ_robotics::DQ_threshold).finished();
    VectorXd v2 = (VectorXd(3)<<1,2,3+DQ_robotics::DQ_threshold).finished();
    VectorXd v3;

    if (v1 == v2)
    {
        std::cout<<"Equal "<<std::endl;
    }else
    {
        std::cout<<"Different "<<std::endl;
    }
    std::cout<<"size v3: "<<v3.size()<<std::endl;




    //std::cout<<"forces: "<<forces.transpose()<<std::endl;
    //std::cout<<"Inertia M:"<<gp_solver.compute_inertia_matrix(q)<<std::endl;
    //std::cout<<"Coriolis V:"<<gp_solver.compute_coriolis_vector(q,q).transpose()<<std::endl;
    //std::cout<<"Grav forces:"<<gp_solver.compute_gravitational_forces_vector(q).transpose()<<std::endl;
}
