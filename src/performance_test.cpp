#include "dqrobotics/solvers/DQ_NewtonEulerSolver.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/solvers/DQ_GaussPrincipleSolver.h>
#include <random>
#include <fstream>

using namespace Eigen;
using namespace DQ_robotics;


VectorXd get_random_vector(const int& size = 7);
double get_average(const std::vector<double>& a);
double get_variance(const std::vector<double>& a);


int main(void)
{

    std::ofstream list_time_gp;
    list_time_gp.open("time_gp.csv");

    std::ofstream list_time_ne;
    list_time_ne.open("time_ne.csv");

    auto robot = std::make_shared<DQ_SerialManipulatorMDH>(FrankaEmikaPandaRobot::dynamics());
    auto gp_solver =  std::make_shared<DQ_GaussPrincipleSolver>();
    auto ne_solver =  std::make_shared<DQ_NewtonEulerSolver>();

    const int trials = 10000;
    const int n = robot->get_dim_configuration_space();
    std::vector<VectorXd>      q_list(trials, VectorXd::Zero(n));
    std::vector<VectorXd>  q_dot_list(trials, VectorXd::Zero(n));
    std::vector<VectorXd> q_ddot_list(trials, VectorXd::Zero(n));
    for (size_t i = 0; i<trials; i++)
    {
        q_list.at(i)      = get_random_vector(n);
        q_dot_list.at(i)  = get_random_vector(n);
        q_ddot_list.at(i) = get_random_vector(n);
    }

    std::vector<double> ts_gp(trials,0);
    //std::vector<
    robot->dynamic_solver(gp_solver);
    for (size_t i = 0; i<trials; i++)
    {
        auto initial_time_ = std::chrono::steady_clock::now();
        auto forces = robot->compute_generalized_forces(q_list.at(i), q_dot_list.at(i), q_ddot_list.at(i));
        auto end{std::chrono::steady_clock::now()};
        auto elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
        ts_gp.at(i) = elapsed_seconds_.count();
        list_time_gp <<ts_gp.at(i)<<'\n';
    }

    std::cout<<"GP Mean time: "<<get_average(ts_gp)*1e3<<" (ms) ±"<<std::sqrt(get_variance(ts_gp))*1e3<<" (ms)"<<std::endl;

    std::vector<double> ts_ne(trials,0);
    robot->dynamic_solver(ne_solver);
    for (size_t i = 0; i<trials; i++)
    {
        auto initial_time_ = std::chrono::steady_clock::now();
        auto forces = robot->compute_generalized_forces(q_list.at(i), q_dot_list.at(i), q_ddot_list.at(i));
        auto end{std::chrono::steady_clock::now()};
        auto elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
        ts_ne.at(i) = elapsed_seconds_.count();
        list_time_ne <<ts_ne.at(i)<<'\n';
    }

    std::cout<<"NE Mean time: "<<get_average(ts_ne)*1e3<<" (ms) ±"<<std::sqrt(get_variance(ts_ne))*1e3<<" (ms)"<<std::endl;

    list_time_gp.close();
    list_time_ne.close();

    /*
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
*/

}

VectorXd get_random_vector(const int& size)
{
    std::random_device seeder;
    const auto seed { seeder.entropy() ? seeder() : time(nullptr) };
    std::mt19937 engine { static_cast<std::mt19937::result_type>(seed)};
    std::uniform_int_distribution<int> distribution { 1, 99};
    auto generator { std::bind(distribution, engine)};
    std::vector<double> values(size);
    std::generate(begin(values), end(values), generator);
    VectorXd v = -50*VectorXd::Ones(size)+M_PI/2*Eigen::Map<VectorXd>(values.data(), values.size());
    return 0.1*v;
}

double get_average(const std::vector<double>& a)
{
    const unsigned long N = a.size();
    double acc = 0;
    for(const double& point : a)
    {
        acc+=point;
    }
    return acc/double(N);
}

double get_variance(const std::vector<double>& a)
{
    const unsigned long N = a.size();
    const double m = get_average(a);

    double acc=0;
    for(const double& point : a)
    {
        acc+=pow(point-m,2);
    }
    return acc/double(N);
}
