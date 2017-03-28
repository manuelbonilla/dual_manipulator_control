
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <boost/scoped_ptr.hpp>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_wrapper/kdl_wrapper.h>

#include <dual_manipulation_control/utils.hpp>


class Kuka_LWR
{
public:
    Kuka_LWR() {};
    ~Kuka_LWR() {};

    struct gains_t
    {
        gains_t(): ik_kp(0.1) {};
        float ik_kp;
    };


    bool init_robot(std::string name_in, KDL::Chain chain, double sampling_rate);
    void getJointSates(const sensor_msgs::JointState::ConstPtr msg);
    void computeControl();
    void setReferences();
    Eigen::VectorXf getTau() {return tau;};
    void computeReferences(Eigen::VectorXf &q, Eigen::VectorXf &qp, Eigen::VectorXf &qpp );
    void initControl();
    void setGains(gains_t gains_in) {gains =  gains_in;};
    void setXReference(KDL::Frame x_des_in) {x_des = x_des_in;};
    Eigen::MatrixXf getJacobian() {return J_;};
    Eigen::MatrixXf getM() {return M;};
    Eigen::VectorXf getC() {return C;};

private:
    Eigen::VectorXf q, qp, qpp, tau, error_q, error_qp, q_ref, qp_ref, qpp_ref;
    std::string name;
    std::vector<int> index;
    KDL::JntArrayAcc joint_msr_states;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
    boost::scoped_ptr<KDL::ChainDynParam> id_solver;
    Eigen::MatrixXf M; //Inertia matrix
    Eigen::VectorXf C;   //Coriolis and Gravitational matrices
    Eigen::VectorXf G;
    Eigen::VectorXf J_;
    int n_joints;
    std::string name_space;
    KDL::Frame x_des;

    float t_total;
    float dt;

    gains_t gains;

    // void gwtTau(Eigen::VectorXf &tau_in) {tau =  tau_in;};

};

bool Kuka_LWR::init_robot(std::string name_in, KDL::Chain chain, double sampling_rate)
{
    name = name_in;
    index = std::vector<int>({0, 1, 6, 2, 3, 4, 5});
    n_joints = chain.getNrOfJoints();
    q =  Eigen::VectorXf::Zero(n_joints);
    qp =  Eigen::VectorXf::Zero(n_joints);
    qpp =  Eigen::VectorXf::Zero(n_joints);
    tau =  Eigen::VectorXf::Zero(n_joints);
    error_q =  Eigen::VectorXf::Zero(n_joints);
    error_qp =  Eigen::VectorXf::Zero(n_joints);
    q_ref =  Eigen::VectorXf::Zero(n_joints);
    qp_ref =  Eigen::VectorXf::Zero(n_joints);
    qpp_ref =  Eigen::VectorXf::Zero(n_joints);

    C =  Eigen::VectorXf::Zero(n_joints);
    M =  Eigen::MatrixXf::Zero(n_joints, n_joints);

    KDL::Vector gravity;
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    id_solver.reset(new KDL::ChainDynParam(chain, gravity) );
    t_total = 0;
    dt = sampling_rate;

    return true;
}

void Kuka_LWR::initControl()
{
    q_ref = q;
}

void Kuka_LWR::getJointSates(const sensor_msgs::JointState::ConstPtr msg)
{
    Eigen::VectorXf q_local = Eigen::VectorXf::Zero(n_joints);
    Eigen::VectorXf qp_local = Eigen::VectorXf::Zero(n_joints);
    Eigen::VectorXf qpp_local = Eigen::VectorXf::Zero(n_joints);
    for (unsigned int i = 0; i < msg->position.size(); ++i)
    {
        q_local(i) = msg->position[index[i]];
        qp_local(i) = msg->velocity[index[i]];
        qpp_local(i) = msg->effort[index[i]];
    }

    q = q_local;
    qp = qp_local;
    qpp = qpp_local;

}

void Kuka_LWR::computeControl()
{
    computeReferences(q_ref, qp_ref, qpp_ref);
    tau = -300. * (q - q_ref) - 1. * (qp - qp_ref);
}

void Kuka_LWR::computeReferences(Eigen::VectorXf &q_local, Eigen::VectorXf &qp_local, Eigen::VectorXf &qpp_local )
{

    KDL::Frame x;
    KDL::JntArrayAcc q_now(n_joints);
    for (int i = 0; i < n_joints; ++i)
    {
        q_now.q(i) = q(i);
        q_now.qdot(i) = qp(i);
    }

    KDL::Jacobian J(n_joints);
    jnt_to_jac_solver->JntToJac(q_now.q, J);
    fk_pos_solver->JntToCart(q_now.q, x);
    KDL::JntSpaceInertiaMatrix M_l; //Inertia matrix
    KDL::JntArray C_l;   //Coriolis and Gravitational matrices

    M_l.resize(n_joints);
    C_l.resize(n_joints);
    for (int i = 0; i < n_joints; ++i)
    {
        C(i) = C_l(i);
    }
    for (int i = 0; i < n_joints; ++i)
    {
        for (int j = 0; j < n_joints; ++j)
        {
            M(i,j) = M_l(i,j);
        }
    }

    id_solver->JntToMass(q_now.q, M_l);
    id_solver->JntToCoriolis(q_now.q, q_now.qdot, C_l);

    KDL::Twist pose_error;

    pose_error.vel = x.p - x_des.p;
    pose_error.rot = 0.5 * (x_des.M.UnitX() * x.M.UnitX() +
                            x_des.M.UnitY() * x.M.UnitY() +
                            x_des.M.UnitZ() * x.M.UnitZ());

    Eigen::VectorXf err = Eigen::VectorXf::Zero(6);
    for (int i = 0; i < 6; ++i)
    {
        err(i) = pose_error(i);
    }

    Eigen::MatrixXf J_pinv;
    J_ = J.data.cast<float>();
    pseudo_inverse(J.data.cast<float>(), J_pinv, true);
    qp_local =  -gains.ik_kp * J_pinv * err;

    for (unsigned  int i = 0; i < n_joints ; i++)
    {
        q_local(i) += dt * qp_local(i);
    }

    qpp_local = Eigen::VectorXf::Zero(n_joints);

    t_total += dt;

}
