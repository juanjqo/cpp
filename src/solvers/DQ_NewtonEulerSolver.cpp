#include <dqrobotics/solvers/DQ_NewtonEulerSolver.h>


namespace DQ_robotics
{

std::vector<DQ> DQ_NewtonEulerSolver::_compute_wrenches_ne(const std::vector<DQ> &xs,
                                                           const std::vector<DQ> &joint_twists,
                                                           const std::vector<DQ> &joint_twists_dot,
                                                           const DQ &gravity)
{
    _forward_recursion(xs, joint_twists, joint_twists_dot);
    _backward_recursion(gravity);
    return wrenches_;
}

void DQ_NewtonEulerSolver::_forward_recursion(const std::vector<DQ> &xs, const std::vector<DQ> &joint_twists, const std::vector<DQ> &joint_twists_dot)
{
    DQ twisti{0};
    DQ twisti_im1{0};
    DQ twisti_dot{0};
    DQ x_0_im1 {1};
    DQ x_0_cim1{1};
    DQ xi {1};
    twists_ = std::vector<DQ>(n_links_, DQ(0));
    twists_dot_ = std::vector<DQ>(n_links_, DQ(0));
    wrenches_= std::vector<DQ>(n_links_, DQ(0));
    frames_x_ci_to_im1_= std::vector<DQ>(n_links_, DQ(0));
    frames_x_im1_to_i_= std::vector<DQ>(n_links_, DQ(0));
    frames_x_0_ci_= std::vector<DQ>(n_links_, DQ(0));

    for(int i=0; i<n_links_;i++) // forward recursion
    {
        //twists_.push_back(DQ(0));
        //twists_dot_.push_back(DQ(0));
        //wrenches_.push_back(DQ(0));
        //frames_x_ci_to_im1_.push_back(DQ(0));
        //frames_x_im1_to_i_.push_back(DQ(0));
        //frames_x_0_ci_.push_back(DQ(0));

        DQ x_0_to_i = xs.at(i); //fkm(q,i);
        DQ x_i_to_ci = DQ(1,0,0,0,0, 0.5*center_of_masses_[i](0),
                          0.5*center_of_masses_[i](1),
                          0.5*center_of_masses_[i](2));
        DQ x_0_ci = x_0_to_i*x_i_to_ci;

        DQ x_ci_to_cim1 = x_0_ci.conj()*x_0_cim1;
        DQ x_ci_to_im1 =  x_0_ci.conj()*x_0_im1;
        frames_x_ci_to_im1_[i] = x_ci_to_im1;
        frames_x_im1_to_i_[i] = xi*x_0_to_i;
        frames_x_0_ci_[i] = x_0_ci;

        twisti = Ad(x_ci_to_cim1, twisti_im1) + Ad(x_ci_to_im1, joint_twists[i]);

        twisti_dot = Ad(x_ci_to_cim1, twisti_dot) + Ad(x_ci_to_im1, joint_twists_dot[i])
                     + cross(-Ad(x_ci_to_im1, joint_twists[i]), Ad(x_ci_to_cim1, twisti_im1));

        twists_.at(i) = twisti;
        twists_dot_.at(i) = twisti_dot;

        //----Updated new values
        x_0_cim1 = x_0_ci;
        x_0_im1 = x_0_to_i;
        twisti_im1 = twisti;
        xi = x_0_to_i.conj();
    }
}

void DQ_NewtonEulerSolver::_backward_recursion(const DQ &gravity)
{
    DQ wrench_i_to_jn1 = DQ(0);
    for (int i=n_links_-1; i>=0;i--) // Backward recursion
    {
        auto twisti = twists_[i];
        auto twisti_dot = twists_dot_[i];

        DQ force_ci = masses_[i]*(twisti_dot.D() + cross(twisti.P(), twisti.D()));
        DQ torque_ci = _M3( inertia_tensors_[i], twisti_dot.P() )+
                       cross(twisti.P(), _M3(inertia_tensors_[i], twisti.P()) );
        DQ wrench_ci = force_ci + E_*torque_ci + masses_[i]*Ad(frames_x_0_ci_[i].P().conj(), -gravity);

        wrenches_[i] = Ad(frames_x_ci_to_im1_[i].conj(), wrench_ci) + Ad(frames_x_im1_to_i_[i], wrench_i_to_jn1 );
        wrench_i_to_jn1 = wrenches_[i];
    }
}

DQ DQ_NewtonEulerSolver::_M3(const MatrixXd &inertia_tensor, const DQ &h)
{
    MatrixXd I = inertia_tensor;
    VectorXd vech = vec3(h);
    double h2 = vech(0);
    double h3 = vech(1);
    double h4 = vech(2);
    DQ ret = i_*(I(0,0)*h2 + I(0,1)*h3 + I(0,2)*h4 ) +
             j_*(I(1,0)*h2 + I(1,1)*h3 + I(1,2)*h4 ) +
             k_*(I(2,0)*h2 + I(2,1)*h3 + I(2,2)*h4 );
    return ret;
}

VectorXd DQ_NewtonEulerSolver::_compute_torques(const VectorXd &q, const VectorXd &q_dot, const VectorXd &q_dot_dot, const bool &fkm_flag)
{
    int n_links = robot_->get_dim_configuration_space();
    std::vector<DQ> xs;
    std::vector<DQ> joint_twists;
    std::vector<DQ> joint_twists_dot;
    VectorXd torques = VectorXd(n_links);

    if (fkm_flag == true)
    {
        xs_ = xs;
        for(int i=0; i<n_links;i++)
        {
            xs_.push_back(DQ(0));
            xs_[i] = robot_->get_base_frame().conj()*robot_->fkm(q,i);
        }
    }

    for(int i=0; i<n_links;i++)
    {
        //xs.push_back(DQ(0));
        joint_twists.push_back(DQ(0));
        joint_twists_dot.push_back(DQ(0));
        joint_twists[i] =     q_dot(i)*serial_manipulator_ ->get_joint_twist(i);
        joint_twists_dot[i] = q_dot_dot(i)*serial_manipulator_->get_joint_twist(i);

    }

    auto wrenches = _compute_wrenches_ne(xs_, joint_twists,
                                                         joint_twists_dot,
                                                         robot_->get_gravity_acceleration());
    for(int i=0; i<n_links;i++)
    {
        torques[i] = serial_manipulator_ ->get_generalized_joint_force(wrenches[i], i);
    }
    return torques;
}

DQ_NewtonEulerSolver::DQ_NewtonEulerSolver(const std::shared_ptr<DQ_Dynamics> &robot):
DQ_DynamicsSolver(robot)
{
    inertia_tensors_ = robot_->get_inertia_tensors();
    center_of_masses_ = robot_->get_center_of_masses();
    masses_ = robot_->get_masses();
    n_links_ = masses_.size();
    n_dim_space_ = robot_->get_dim_configuration_space();
    serial_manipulator_ = std::dynamic_pointer_cast<DQ_SerialManipulator>(robot_);
    if(!serial_manipulator_)
    {
        throw std::runtime_error("DQ_NewtonEulerSolver only supports serial manipulators");
    }
}

VectorXd DQ_NewtonEulerSolver::compute_generalized_forces(const VectorXd &q,
                                                          const VectorXd &q_dot,
                                                          const VectorXd &q_dot_dot)
{
    return _compute_torques(q, q_dot, q_dot_dot, true);

}

MatrixXd DQ_NewtonEulerSolver::compute_inertia_matrix(const VectorXd &q)
{
    const int size = q.size();
    VectorXd zeros = VectorXd::Zero(size);
    MatrixXd I =  MatrixXd::Identity(size, size);
    MatrixXd M = MatrixXd(size, size);
    for (int i=0;i<size;i++)
    {
        M.block(0,i, size, 1) = _compute_torques(q, zeros, I.col(i), false);  // I.col(i)

    }
    inertia_matrix_ = M;
    return inertia_matrix_;

}

VectorXd DQ_NewtonEulerSolver::compute_coriolis_vector(const VectorXd &q, const VectorXd &q_dot)
{

    VectorXd zeros = VectorXd::Zero(q.size());
    auto gravity = robot_->get_gravity_acceleration();
    robot_->set_gravity_acceleration(DQ(0)); // Gravity must be set to zero for the calculation of 'c(q,q_dot)' and 'M(q)'.
    coriolis_vector_ = _compute_torques(q, q_dot, zeros, false);
    robot_->set_gravity_acceleration(gravity);
    return coriolis_vector_;
}

VectorXd DQ_NewtonEulerSolver::compute_gravitational_forces_vector(const VectorXd &q)
{
    //auto gravity = robot_->get_gravity_acceleration();
    //robot_->set_gravity_acceleration(DQ(0));
    const int size = q.size();
    VectorXd zeros = VectorXd::Zero(size);
    VectorXd torques = _compute_torques(q, zeros, zeros, true);
    //robot_->set_gravity_acceleration(gravity);
    return torques;
}

}
