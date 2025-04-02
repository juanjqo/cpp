#include <dqrobotics/solvers/DQ_GaussPrincipleSolver.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

DQ_GaussPrincipleSolver::DQ_GaussPrincipleSolver(const std::shared_ptr<DQ_Dynamics> &robot):
    DQ_DynamicsSolver(robot)
{
    //dh_chain_  = std::dynamic_pointer_cast<DQ_SerialManipulatorDH>(robot_);
    serial_manipulator_ = std::dynamic_pointer_cast<DQ_SerialManipulator>(robot_);
    serial_whole_body_ = std::dynamic_pointer_cast<DQ_SerialWholeBody>(robot_);
    holonomic_base_ = std::dynamic_pointer_cast<DQ_HolonomicBase>(robot_);



    if (serial_manipulator_)
    {
        std::cerr<<"Cast to Serial Maipulator performed!"<<std::endl;
        robot_type_ = ROBOT_TYPE::SERIAL_MANIPULATOR;
    }else if (serial_whole_body_)
    {
        robot_type_ = ROBOT_TYPE::SERIAL_WHOLE_BODY;
        std::cerr<<"Cast to Serial Whole Body performed!"<<std::endl;
    }else if(holonomic_base_)
    {
        robot_type_ = ROBOT_TYPE::HOLONOMIC_BASE;
        std::cerr<<"Cast to Serial Holonomic Base!"<<std::endl;
    }else{
        throw std::runtime_error("Wrong robot type casting in DQ_GaussPrincipleSolver.");
    }

    inertia_tensors_ = robot_->get_inertia_tensors();
    center_of_masses_ = robot_->get_center_of_masses();
    masses_ = robot_->get_masses();
    n_links_ = masses_.size();
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
    _initialize_vectors();
}

VectorXd DQ_GaussPrincipleSolver::compute_generalized_forces(const VectorXd &q, const VectorXd &q_dot, const VectorXd &q_dot_dot)
{
    _compute_twist_jacobian_derivatives(q, q_dot);

    return inertia_matrix_gp_*q_dot_dot + coriolis_vector_gp_ + gravitational_forces_gp_;
}

bool DQ_GaussPrincipleSolver::_update_robot_configuration(const VectorXd &q)
{
    bool update_performed = false;
    if (q_.size() != 0)
    {
        if (q_ != q)
        {
            q_ = q;
            update_performed = true;
        }
    }else{
        q_ = q;
        update_performed = true;
    }
    return update_performed;
}

bool DQ_GaussPrincipleSolver::_update_robot_configuration_velocities(const VectorXd &q_dot)
{
    bool update_performed = false;
    if (dq_.size() != 0)
    {
        if (dq_ != q_dot)
        {
            dq_ = q_dot;
            update_performed = true;
        }
    }else{
        dq_ = q_dot;
        update_performed = true;
    }
    return update_performed;
}

void DQ_GaussPrincipleSolver::_initialize_vectors()
{
    MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_links_);
    MatrixXd J_aux = MatrixXd::Zero(8, n_links_);
    MatrixXd J_aux_dot = MatrixXd::Zero(8, n_links_);

    J_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    J_dot_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    Jecom_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    Jecom_dot_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    xcoms_ = std::vector<DQ>(n_links_, DQ(1));
    xs_  = std::vector<DQ>(n_links_, DQ(1));
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
    std::cout<<"------"<<std::endl;

    //DQ_GaussPrinciple::_compute_euler_lagrange_gp(xs, Js, Js_dot, q_dot, robot_->get_gravity_acceleration());

}

MatrixXd DQ_GaussPrincipleSolver::twist_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    return 2*hamiplus8(pose.conj())*pose_jacobian;
}

MatrixXd DQ_GaussPrincipleSolver::twist_jacobian_derivative(const MatrixXd &pose_jacobian,
                                                      const MatrixXd &pose_jacobian_derivative,
                                                      const DQ &pose, const VectorXd &q_dot)
{
    return 2*hamiplus8(DQ(C8()*pose_jacobian*q_dot))*pose_jacobian+2*hamiplus8(pose.conj())*pose_jacobian_derivative;
}

void DQ_GaussPrincipleSolver::_compute_twist_jacobians(const VectorXd &q)
{
    if (_update_robot_configuration(q))
    {
        MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_links_);
        MatrixXd J_aux = MatrixXd::Zero(8, n_links_);
        MatrixXd J_aux_dot = MatrixXd::Zero(8, n_links_);
        MatrixXd Z = MatrixXd(4, 4);
        inertia_matrix_gp_ = MatrixXd::Zero(n_links_,n_links_);
        gravitational_forces_gp_ = VectorXd::Zero(n_links_);

        const DQ &gravity = robot_->get_gravity_acceleration();

        for(int i=0; i<n_links_;i++)
        {
            xs_.at(i) = robot_->fkm(q,i);
            xcoms_.at(i) = xs_.at(i)*DQ(1,0,0,0,0, 0.5*center_of_masses_[i](0),
                                                    0.5*center_of_masses_[i](1),
                                                    0.5*center_of_masses_[i](2));

            J_aux.block(0,0,8,i+1) = robot_->pose_jacobian(q,i);


            J_.at(i) = J_aux;
            Jecom_.at(i) =     twist_jacobian(haminus8(xs_.at(i).conj()*xcoms_.at(i))*J_.at(i), xcoms_.at(i));
            inertia_matrix_gp_ = inertia_matrix_gp_ + Jecom_.at(i).transpose()*Psi_.at(i)*Jecom_.at(i);

            Z =  hamiplus4(xcoms_.at(i).P().conj())*haminus4(xcoms_.at(i).P());
            gravitational_forces_gp_ = gravitational_forces_gp_ + Jecom_.at(i).block(4,0,4,n_links_).transpose()*Z*vec4(Psi_.at(i)(7,7)*gravity);
        }
    }
}

void DQ_GaussPrincipleSolver::_compute_twist_jacobian_derivatives(const VectorXd &q,
                                                                  const VectorXd &q_dot)
{
    _compute_twist_jacobians(q);

    if (_update_robot_configuration_velocities(q_dot))
    {
        MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_links_);
        MatrixXd J_aux_dot = MatrixXd::Zero(8, n_links_);
        coriolis_vector_gp_ = VectorXd::Zero(n_links_);
        MatrixXd S8 = MatrixXd::Zero(8, 8);


        for(int i=0; i<n_links_;i++)
        {
            J_aux_dot.block(0,0,8,i+1) = robot_->pose_jacobian_derivative(q, q_dot,i);
            J_dot_.at(i) = J_aux_dot;
            Jecom_dot_.at(i) = twist_jacobian_derivative(haminus8(xs_.at(i).conj()*xcoms_.at(i))*J_.at(i),
                                                         haminus8(xs_.at(i).conj()*xcoms_.at(i))*J_dot_.at(i),
                                                         xcoms_.at(i), q_dot);

            MatrixXd W_vec = Jecom_.at(i).block(0,0,4,n_links_)*q_dot;
            MatrixXd S4 = 0.5*(hamiplus4(DQ(W_vec))-haminus4(DQ(W_vec)));
            MatrixXd S_sci = 0.5*(hamiplus4(DQ(Psi_.at(i).block(0,0,4,4)*W_vec))-haminus4(DQ(Psi_.at(i).block(0,0,4,4)*W_vec)));
            S8.block(0, 0, 4,4) = -S_sci;
            S8.block(4, 4, 4,4) = Psi_.at(i)(7,7)*S4;
            coriolis_vector_gp_  = coriolis_vector_gp_
                                  + (Jecom_.at(i).transpose()*Psi_.at(i)*Jecom_dot_.at(i)+
                                     Jecom_.at(i).transpose()*S8*Jecom_.at(i))*q_dot;


        }
    }
}

MatrixXd DQ_GaussPrincipleSolver::compute_inertia_matrix(const VectorXd& q)
{
    _compute_twist_jacobians(q);
    return inertia_matrix_gp_;
}

VectorXd DQ_GaussPrincipleSolver::compute_coriolis_vector(const VectorXd&q, const VectorXd& q_dot)
{
    _compute_twist_jacobian_derivatives(q, q_dot);
    return coriolis_vector_gp_;
}

VectorXd DQ_GaussPrincipleSolver::compute_gravitational_forces_vector(const VectorXd& q)
{
    _compute_twist_jacobians(q);
    return gravitational_forces_gp_;
}



}
