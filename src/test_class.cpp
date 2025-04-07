#include "dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h"
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

    std::vector<Matrix<double, 3,3>> rinertia_tensors = {MatrixXd::Identity(3,3)};
    std::vector<DQ> center_of_masses = {DQ(1,2,3)};
    std::vector<double> masses = {1.0};

    auto robot2 = std::make_shared<DQ_HolonomicBase>(rinertia_tensors, center_of_masses, masses);


    auto robot3 = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::dynamics());

    auto robot4 = std::make_shared<DQ_DifferentialDriveRobot>(0.1, 0.3);

    auto gp_solver = DQ_GaussPrincipleSolver(robot3);

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
    VectorXd q3 = (VectorXd(3) << 1,2,3).finished();


    auto initial_time_ = std::chrono::steady_clock::now();

    //q = q3;


    auto forces = gp_solver.compute_generalized_forces(q, q, q);


    auto end{std::chrono::steady_clock::now()};
    auto elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
    std::cout<<"elapsed time: "<<elapsed_seconds_.count()*1e3<<" ms"<<std::endl;


    //
    initial_time_ = std::chrono::steady_clock::now();
    auto M = gp_solver.compute_inertia_matrix(q);
    end = std::chrono::steady_clock::now();
    elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
    std::cout<<"elapsed time: "<<elapsed_seconds_.count()*1e3<<" ms"<<std::endl;


    initial_time_ = std::chrono::steady_clock::now();
    auto C = gp_solver.compute_coriolis_vector(q,q);
    end = std::chrono::steady_clock::now();
    elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
    std::cout<<"elapsed time: "<<elapsed_seconds_.count()*1e3<<" ms"<<std::endl;

    initial_time_ = std::chrono::steady_clock::now();
    auto g = gp_solver.compute_gravitational_forces_vector(q);
    end = std::chrono::steady_clock::now();
    elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
    std::cout<<"elapsed time: "<<elapsed_seconds_.count()*1e3<<" ms"<<std::endl;


    //std::cout<<"forces: "<<forces.transpose()<<std::endl;
    //std::cout<<"Inertia M:"<<gp_solver.compute_inertia_matrix(q)<<std::endl;
    //std::cout<<"Coriolis V:"<<gp_solver.compute_coriolis_vector(q,q).transpose()<<std::endl;
    //std::cout<<"Grav forces:"<<gp_solver.compute_gravitational_forces_vector(q).transpose()<<std::endl;
}
