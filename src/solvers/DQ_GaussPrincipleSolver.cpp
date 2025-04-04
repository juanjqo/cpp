#include <dqrobotics/solvers/DQ_GaussPrincipleSolver.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

DQ_GaussPrincipleSolver::DQ_GaussPrincipleSolver(const std::shared_ptr<DQ_Dynamics> &robot):
    DQ_DynamicsSolver(robot)
{
    serial_manipulator_ = std::dynamic_pointer_cast<DQ_SerialManipulator>(robot_);
    serial_whole_body_ = std::dynamic_pointer_cast<DQ_SerialWholeBody>(robot_);
    holonomic_base_ = std::dynamic_pointer_cast<DQ_HolonomicBase>(robot_);
    differential_base_ = std::dynamic_pointer_cast<DQ_DifferentialDriveRobot>(robot_);



    if (serial_manipulator_)
    {
        robot_type_ = ROBOT_TYPE::SERIAL_MANIPULATOR;
    }else if (serial_whole_body_)
    {
        robot_type_ = ROBOT_TYPE::SERIAL_WHOLE_BODY;
        throw std::runtime_error("Serial Whole body robots are unsupported!");
    }else if(holonomic_base_)
    {
        if(differential_base_)
            throw std::runtime_error("Differential robots are unsupported!");
        robot_type_ = ROBOT_TYPE::HOLONOMIC_BASE;
    }else{
        throw std::runtime_error("Wrong robot type in DQ_GaussPrincipleSolver.");
    }

    inertia_tensors_ = robot_->get_inertia_tensors();
    center_of_masses_ = robot_->get_center_of_masses();
    masses_ = robot_->get_masses();
    n_links_ = masses_.size();
    n_dim_space_ = robot_->get_dim_configuration_space();
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
    _initialize_variables();
}

VectorXd DQ_GaussPrincipleSolver::compute_generalized_forces(const VectorXd &q, const VectorXd &q_dot, const VectorXd &q_dot_dot)
{
    _compute_robot_dynamics(q, q_dot);

    return inertia_matrix_gp_*q_dot_dot + coriolis_vector_gp_ + gravitational_forces_gp_;
}

bool DQ_GaussPrincipleSolver::_update_configuration(const VectorXd &q)
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

bool DQ_GaussPrincipleSolver::_update_configuration_velocities(const VectorXd &q_dot)
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

void DQ_GaussPrincipleSolver::_initialize_variables()
{
    MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_dim_space_);
    MatrixXd J_aux = MatrixXd::Zero(8, n_dim_space_);
    MatrixXd J_aux_dot = MatrixXd::Zero(8, n_dim_space_);


    J_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    J_dot_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    Jecom_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    Jecom_dot_ = std::vector<MatrixXd>(n_links_, zeros_8_nlinks);
    xcoms_ = std::vector<DQ>(n_links_, DQ(1));
    xs_  = std::vector<DQ>(n_links_, DQ(1));
}

DQ DQ_GaussPrincipleSolver::_get_fkm(const VectorXd &q, const int &to_ith_link)
{
    switch (robot_type_) {

    case ROBOT_TYPE::SERIAL_MANIPULATOR:
        return robot_->fkm(q, to_ith_link);
    case ROBOT_TYPE::SERIAL_WHOLE_BODY:
        throw std::runtime_error("Serial Whole body robots are not supported yet!");
    case ROBOT_TYPE::HOLONOMIC_BASE:
        return robot_->fkm(q,2);
    }
}

MatrixXd DQ_GaussPrincipleSolver::_get_pose_jacobian(const VectorXd &q, const int &to_ith_link)
{
    MatrixXd J_aux = MatrixXd::Zero(8, n_dim_space_);

    switch (robot_type_) {
    case ROBOT_TYPE::SERIAL_MANIPULATOR:
        J_aux.block(0,0,8,to_ith_link+1) = robot_->pose_jacobian(q, to_ith_link);
        return J_aux;
    case ROBOT_TYPE::SERIAL_WHOLE_BODY:
        throw std::runtime_error("Serial Whole body robots are not supported yet!");
    case ROBOT_TYPE::HOLONOMIC_BASE:
        J_aux.block(0,0,8,n_dim_space_) = robot_->pose_jacobian(q,2);//robot_->pose_jacobian(q,i);
        return J_aux;
    }
}

MatrixXd DQ_GaussPrincipleSolver::_get_pose_jacobian_derivative(const VectorXd &q,
                                                                const VectorXd &q_dot,
                                                                const int &to_ith_link)
{
    MatrixXd J_aux_dot = MatrixXd::Zero(8, n_dim_space_);
    switch (robot_type_){
    case ROBOT_TYPE::SERIAL_MANIPULATOR:
         J_aux_dot.block(0,0,8,to_ith_link+1) = robot_->pose_jacobian_derivative(q, q_dot, to_ith_link);
         return J_aux_dot;
    case ROBOT_TYPE::SERIAL_WHOLE_BODY:
        throw std::runtime_error("Serial Whole body robots are not supported yet!");
    case ROBOT_TYPE::HOLONOMIC_BASE:
        J_aux_dot.block(0,0,8,n_dim_space_) = robot_->pose_jacobian_derivative(q, q_dot,2);
        return J_aux_dot;
    }
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

void DQ_GaussPrincipleSolver::_compute_robot_dynamics_without_coriolis_effect(const VectorXd &q)
{
    if (_update_configuration(q))
    {

        MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_dim_space_);
        MatrixXd J_aux = MatrixXd::Zero(8, n_dim_space_);
        MatrixXd J_aux_dot = MatrixXd::Zero(8, n_dim_space_);
        MatrixXd Z = MatrixXd(4, 4);
        inertia_matrix_gp_ = MatrixXd::Zero(n_dim_space_,n_dim_space_);
        gravitational_forces_gp_ = VectorXd::Zero(n_dim_space_);

        const DQ &gravity = robot_->get_gravity_acceleration();

        for(int i=0; i<n_links_;i++)
        {
            xs_.at(i) = _get_fkm(q, i);//robot_->fkm(q,i);
            xcoms_.at(i) = xs_.at(i)*DQ(1,0,0,0,0, 0.5*center_of_masses_[i](0),
                                                    0.5*center_of_masses_[i](1),
                                                    0.5*center_of_masses_[i](2));

            //J_aux.block(0,0,8,i+1) = _get_pose_jacobian(q,i);//robot_->pose_jacobian(q,i);
            //J_.at(i) = J_aux;
            J_.at(i) = _get_pose_jacobian(q,i);

            Jecom_.at(i) =     twist_jacobian(haminus8(xs_.at(i).conj()*xcoms_.at(i))*J_.at(i), xcoms_.at(i));
            inertia_matrix_gp_ = inertia_matrix_gp_ + Jecom_.at(i).transpose()*Psi_.at(i)*Jecom_.at(i);

            Z =  hamiplus4(xcoms_.at(i).P().conj())*haminus4(xcoms_.at(i).P());
            //MatrixXd Ja = (Jecom_.at(i).block(4,0,4,n_dim_space_)).transpose();
            gravitational_forces_gp_ = gravitational_forces_gp_ + (Jecom_.at(i).block(4,0,4,n_dim_space_)).transpose()*Z*vec4(Psi_.at(i)(7,7)*gravity);
        }
        gravitational_forces_gp_ = -1*gravitational_forces_gp_;
    }
}

void DQ_GaussPrincipleSolver::_compute_robot_dynamics(const VectorXd &q, const VectorXd &q_dot)
{
    _compute_robot_dynamics_without_coriolis_effect(q);

    if (_update_configuration_velocities(q_dot))
    {
        MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_dim_space_);
        coriolis_vector_gp_ = VectorXd::Zero(n_dim_space_);
        MatrixXd S8 = MatrixXd::Zero(8, 8);


        for(int i=0; i<n_links_;i++)
        {
            //J_aux_dot.block(0,0,8,i+1) = _get_pose_jacobian_derivative(q, q_dot, i); //robot_->pose_jacobian_derivative(q, q_dot,i);
            J_dot_.at(i) = _get_pose_jacobian_derivative(q, q_dot, i);//J_aux_dot;
            Jecom_dot_.at(i) = twist_jacobian_derivative(haminus8(xs_.at(i).conj()*xcoms_.at(i))*J_.at(i),
                                                         haminus8(xs_.at(i).conj()*xcoms_.at(i))*J_dot_.at(i),
                                                         xcoms_.at(i), q_dot);

            MatrixXd W_vec = Jecom_.at(i).block(0,0,4,n_dim_space_)*q_dot;
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
    _compute_robot_dynamics_without_coriolis_effect(q);
    return inertia_matrix_gp_;
}

VectorXd DQ_GaussPrincipleSolver::compute_coriolis_vector(const VectorXd&q, const VectorXd& q_dot)
{
    _compute_robot_dynamics(q, q_dot);
    return coriolis_vector_gp_;
}

VectorXd DQ_GaussPrincipleSolver::compute_gravitational_forces_vector(const VectorXd& q)
{
    _compute_robot_dynamics_without_coriolis_effect(q);
    return gravitational_forces_gp_;
}



}
