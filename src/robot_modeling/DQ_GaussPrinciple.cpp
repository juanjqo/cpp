#include <dqrobotics/robot_modeling/DQ_GaussPrinciple.h>

namespace DQ_robotics
{

void DQ_GaussPrinciple::_compute_euler_lagrange_gp(const std::vector<DQ> &xs, const std::vector<MatrixXd> &Js,
                                                   const std::vector<MatrixXd> &Js_dot, const VectorXd &q_dot, const DQ &gravity)
{
    MatrixXd zeros_8_nlinks = MatrixXd::Zero(8, n_links_);
    MatrixXd J_aux = MatrixXd::Zero(8, n_links_);
    MatrixXd J_aux_dot = MatrixXd::Zero(8, n_links_);

    for(int i=0; i<n_links_;i++)
    {
        Jecom_.push_back(zeros_8_nlinks);
        Jecom_dot_.push_back(zeros_8_nlinks);
        xcoms_.push_back(DQ(1));
        xs_.push_back(DQ(1));

        DQ x = xs[i];
        xs_.at(i) = x;
        DQ xcom = x*DQ(1,0,0,0,0, 0.5*center_of_masses_[i](0),
                         0.5*center_of_masses_[i](1),
                         0.5*center_of_masses_[i](2));
        xcoms_.at(i) = xcom;
        J_aux.block(0,0,8,i+1) = Js[i];
        J_aux_dot.block(0,0,8,i+1) = Js_dot[i];

        MatrixXd J = J_aux;
        MatrixXd J_dot = J_aux_dot;



        Jecom_[i] =     twist_jacobian(haminus8(x.conj()*xcom)*J, xcom);
        Jecom_dot_[i] = twist_jacobian_derivative( haminus8(x.conj()*xcom)*J,
                                                                 haminus8(x.conj()*xcom)*J_dot, xcom, q_dot);
    }
    inertia_matrix_gp_ =       DQ_GaussPrinciple::_compute_inertia_matrix(Jecom_, Psi_);
    coriolis_vector_gp_ =      DQ_GaussPrinciple::_compute_coriolis_vector(Jecom_, Psi_, Jecom_dot_, q_dot);
    gravitational_forces_gp_ = DQ_GaussPrinciple::_compute_gravitational_forces(Jecom_, Psi_, xcoms_, gravity);
}


MatrixXd DQ_GaussPrinciple::twist_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    return 2*hamiplus8(pose.conj())*pose_jacobian;
}

MatrixXd DQ_GaussPrinciple::twist_jacobian_derivative(const MatrixXd &pose_jacobian,
                                                      const MatrixXd &pose_jacobian_derivative,
                                                      const DQ &pose, const VectorXd &q_dot)
{
    return 2*hamiplus8(DQ(C8()*pose_jacobian*q_dot))*pose_jacobian+2*hamiplus8(pose.conj())*pose_jacobian_derivative;
}

MatrixXd DQ_GaussPrinciple::_compute_inertia_matrix(const std::vector<MatrixXd> &Jecom, std::vector<Matrix<double, 8,8>> &Psi)

{
    int n_links = Psi.size();
    MatrixXd inertia_matrix = MatrixXd::Zero(n_links,n_links);
    for(int i=0; i<n_links;i++)
    {
        inertia_matrix = inertia_matrix + Jecom[i].transpose()*Psi[i]*Jecom[i];
    }
    return inertia_matrix;
}

VectorXd DQ_GaussPrinciple::_compute_coriolis_vector(const std::vector<MatrixXd> &Jecom, std::vector<Matrix<double, 8,8>> &Psi,
                                                     const std::vector<MatrixXd> &Jecom_dot, const VectorXd &q_dot)

{
    int n_links = Psi.size();
    VectorXd coriolis_vector = VectorXd::Zero(n_links);
    MatrixXd S8 = MatrixXd::Zero(8, 8);
    for(int i=0; i<n_links;i++)
    {
        MatrixXd W_vec = Jecom[i].block(0,0,4,n_links)*q_dot;
        MatrixXd S4 = 0.5*(hamiplus4(DQ(W_vec))-haminus4(DQ(W_vec)));
        MatrixXd S_sci = 0.5*(hamiplus4(DQ(Psi[i].block(0,0,4,4)*W_vec))-haminus4(DQ(Psi[i].block(0,0,4,4)*W_vec)));
        S8.block(0, 0, 4,4) = -S_sci;
        S8.block(4, 4, 4,4) = Psi[i](7,7)*S4;
        coriolis_vector  = coriolis_vector  + (Jecom[i].transpose()*Psi[i]*Jecom_dot[i]+
                                             Jecom[i].transpose()*S8*Jecom[i])*q_dot;
    }
    return coriolis_vector;
}

VectorXd DQ_GaussPrinciple::_compute_gravitational_forces(const std::vector<MatrixXd> &Jecom,
                                                          std::vector<Matrix<double, 8,8>> &Psi,
                                                          const std::vector<DQ> &xcoms,
                                                          const DQ &gravity)
{
    int n_links = Psi.size();
    VectorXd gravitational_forces = VectorXd::Zero(n_links);
    MatrixXd Z = MatrixXd(4, 4);
    for(int i=0; i<n_links;i++)
    {
        DQ xcom = xcoms.at(i);
        Z =  hamiplus4(xcom.P().conj())*haminus4(xcom.P());
        gravitational_forces = gravitational_forces + Jecom[i].block(4,0,4,n_links).transpose()*Z*vec4(Psi[i](7,7)*gravity);

    }
    return -1*gravitational_forces;
}


}
