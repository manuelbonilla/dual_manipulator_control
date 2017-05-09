#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
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
        gains_t(): ik_kp(0.1), ik_kp_2(0.1) {};
        float ik_kp;
        float ik_kp_2;
    };


    bool init_robot(std::string name_in, KDL::Chain chain, double sampling_rate);
    void getJointSates(const sensor_msgs::JointState::ConstPtr msg);
    void getForceTorqueStates(const geometry_msgs::WrenchStamped::ConstPtr msg);
    void computeControl();
    void setReferences();
    Eigen::VectorXf getTau() {return tau;};
    void computeReferences(Eigen::VectorXf &q, Eigen::VectorXf &qp, Eigen::VectorXf &qpp );
    void initControl();
    void stopControl() {initControl();};
    void setGains(gains_t gains_in) {gains =  gains_in;};
    void setXReference(KDL::Frame x_des_in) {x_des = x_des_in;};
    void setJointStates(Eigen::MatrixXf q, Eigen::MatrixXf qp, Eigen::MatrixXf qpp);
    void setWrenchDesired(Eigen::VectorXf f) {ft_d = f;};
    void saturateJointVelocities(Eigen::VectorXf &qp, float percentage = 1.0);
    void saturateJointPositions( Eigen::VectorXf &q, double soft_limit = 2.0 * M_PI / 180.0 );
    void setX2Reference(KDL::Frame x_des_in) {x_des_2 = x_des_in; enabled_second_task = true;};
    Eigen::MatrixXf getJacobian() {return J_;};
    Eigen::MatrixXf getM() {return M;};
    Eigen::VectorXf getC() {return C;};
    Eigen::VectorXf getErrors() {return error_q;};
    Eigen::VectorXf getCartErr() {return CartErr;};
    Eigen::MatrixXf getEEPose() {return EEPose;};
    Eigen::MatrixXf getXReference() {return EEReference;};
    void enableForceControl(float a) {enable_force_control = a;};
    Eigen::VectorXf q, qp, qpp, tau, error_q, error_qp, q_ref, qp_ref, qpp_ref, ft_m, ft_d;
    int first_step;

private:

    std::string name;
    std::vector<int> index;
    KDL::JntArrayAcc joint_msr_states;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
    boost::scoped_ptr<KDL::ChainDynParam> id_solver;
    Eigen::MatrixXf M; //Inertia matrix
    Eigen::VectorXf C;   //Coriolis and Gravitational matrices
    Eigen::VectorXf G;
    Eigen::VectorXf CartErr;
    Eigen::MatrixXf J_;
    int n_joints;
    std::string name_space;
    KDL::Frame x_des, x_des_2;
    Eigen::MatrixXf Adj;
    Eigen::MatrixXf Adj_skew;
    Eigen::MatrixXf Adj_3;
    Eigen::MatrixXf EEPose;
    Eigen::MatrixXf EEReference;
    float enable_force_control;
    bool enabled_second_task;

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
    CartErr =  Eigen::VectorXf::Zero(6);
    ft_m = Eigen::VectorXf::Zero(6);
    Adj = Eigen::MatrixXf::Zero(6, 6);
    Adj_skew = Eigen::MatrixXf::Zero(3, 3);
    Adj_3 = Eigen::MatrixXf::Zero(3, 3);

    C =  Eigen::VectorXf::Zero(n_joints);
    M =  Eigen::MatrixXf::Zero(n_joints, n_joints);
    J_ = Eigen::MatrixXf::Zero(6, n_joints);
    EEPose = Eigen::MatrixXf::Identity(4, 4);
    EEReference = Eigen::MatrixXf::Identity(4, 4);
    KDL::Vector gravity;
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));
    fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
    id_solver.reset(new KDL::ChainDynParam(chain, gravity) );
    t_total = 0;
    dt = sampling_rate;
    enable_force_control = 0;
    enabled_second_task = false;

    return true;
}

void Kuka_LWR::initControl()
{
    q_ref = q;
    // std::cout << "Initializing control" << std::endl;
    // std::cout << "q: [" << q_ref.transpose() << std::endl;
    KDL::JntArrayAcc q_now(n_joints);
    for (int i = 0; i < n_joints; ++i)
    {
        q_now.q(i) = q(i);
        q_now.qdot(i) = qp(i);
    }
    fk_pos_solver->JntToCart(q_now.q, x_des);
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

void Kuka_LWR::getForceTorqueStates(const geometry_msgs::WrenchStamped::ConstPtr msg)
{
    ft_m(0) = msg->wrench.force.x; /*Force measured in sensor frame*/
    ft_m(1) = msg->wrench.force.y;
    ft_m(2) = msg->wrench.force.z;
    ft_m(3) = msg->wrench.torque.x;
    ft_m(4) = msg->wrench.torque.y;
    ft_m(5) = msg->wrench.torque.z;
}

void Kuka_LWR::computeControl()
{


    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EEReference(i, j) = (float)x_des.M(i, j);
        }
    }
    EEReference(0, 3) = x_des.p(0); EEReference(1, 3) = x_des.p(1); EEReference(2, 3) = x_des.p(2);


    computeReferences(q_ref, qp_ref, qpp_ref);

    Eigen::MatrixXf Jt(7, 6);
    Jt = J_.transpose();
    Eigen::MatrixXf k1(6, 6);
    // k2 = Eigen::MatrixXf::Zero(6,6);

    k1 << 1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;
    k1 = enable_force_control * k1;

    Eigen::VectorXf twist_last(6);
    Eigen::VectorXf twist(6);
    twist = J_ * qp;
    Eigen::VectorXf int_err_f(6);

    if (first_step) {
        twist_last = twist;
        first_step = 0;
    }

    Eigen::VectorXf pos_angle(6);
    pos_angle = (twist + twist_last) * dt;

    double angX = pos_angle(3);
    double angY = pos_angle(4);
    double angZ = pos_angle(5);

    Eigen::MatrixXf Rx(3, 3), Ry(3, 3), Rz(3, 3);
    Rz << cos(angZ), sin(angZ), 0,
    -sin(angZ), cos(angZ), 0,
    0, 0, 1;
    Ry << cos(angY), 0, -sin(angY),
    0, 1, 0,
    sin(angY), 0, cos(angY);
    Rx << 1, 0, 0,
    0, cos(angX), sin(angX),
    0, -sin(angX), cos(angX);

    Adj_3 = Rz * Ry * Rx;

    double posX, posY, posZ;
    posX = pos_angle(0);
    posY = pos_angle(1);
    posZ = pos_angle(2);

    Adj_skew << 0, -posZ, posY,
             posZ, 0, -posX,
             -posY, posX, 0;

    Adj.topLeftCorner(3, 3) = Adj_3;
    Adj.topRightCorner(3, 3) = Adj_skew;
    Adj.bottomLeftCorner(3, 3) = Eigen::MatrixXf::Zero(3, 3);
    Adj.bottomRightCorner(3, 3) = Adj_3;

    Eigen::VectorXf F(6);
    F = Adj * ft_m;

    Eigen::VectorXf Fdes(6);
    Fdes << 0, 0, 0.1, 0, 0, 0;

    tau = -300. * (q - q_ref) - 1. * (qp - qp_ref) + Jt * (k1 * (F - Fdes) /*+ k2*int_err_f*/);

    error_q = q - q_ref;
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

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            EEPose(i, j) = (float)x.M(i, j);
        }
    }
    EEPose(0, 3) = x.p(0); EEPose(1, 3) = x.p(1); EEPose(2, 3) = x.p(2);


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
            M(i, j) = M_l(i, j);
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
        CartErr(i) = err(i);
    }

    Eigen::MatrixXf J_pinv;
    J_ = J.data.cast<float>();
    pseudo_inverse(J.data.cast<float>(), J_pinv, true);

    qp_local = Eigen::VectorXf::Zero(n_joints);
    qp_local =  -gains.ik_kp * J_pinv * err;

    ///// Second Task
    if (enabled_second_task)
    {
        KDL::Frame x_2;
        fk_pos_solver->JntToCart(q_now.q, x_2, 6);

        KDL::Jacobian J_2;
        J_2.resize(n_joints);
        jnt_to_jac_solver->JntToJac(q_now.q, J_2, 6);

        Eigen::Matrix<float, 7, 7> P;
        P =  Eigen::Matrix<float, 7, 7>::Identity() - J_pinv * J_;

        Eigen::Matrix<float, 7, 1> qp_null;
        Eigen::MatrixXf J_pinv_2;

        Eigen::Matrix<float, 3, 7> J_2_short = Eigen::Matrix<float, 3, 7>::Zero();
        J_2_short = J_2.data.block<3, 7>(0, 0).cast<float>();
        pseudo_inverse(J_2_short, J_pinv_2, true);
        Eigen::Matrix<float, 7, 3> NullSpace = Eigen::Matrix<float, 7, 3>::Zero();
        NullSpace = P * J_pinv_2;

        KDL::Twist x_err_2;
        x_err_2.vel = x_2.p - x_des_2.p;

        Eigen::MatrixXf x_err_2_eigen = Eigen::MatrixXf::Zero(3, 1);
        x_err_2_eigen << x_err_2.vel(0) * 0., x_err_2.vel(1), x_err_2.vel(2);

        qp_null = -gains.ik_kp_2 * NullSpace * x_err_2_eigen; //removed scaling factor of .7

        for (int i = 0; i < J_pinv_2.rows(); i++)
        {
            qp_local(i) += qp_null(i); //removed scaling factor of .7
        }
    }

    saturateJointVelocities(qp_local);

    for (unsigned  int i = 0; i < n_joints ; i++)
    {
        q_local(i) += dt * qp_local(i);
    }

    qpp_local = Eigen::VectorXf::Zero(n_joints);
    saturateJointPositions(q_local);

    t_total += dt;

}


void Kuka_LWR::setJointStates(Eigen::MatrixXf q, Eigen::MatrixXf qp, Eigen::MatrixXf qpp)
{

}

void Kuka_LWR::saturateJointVelocities(Eigen::VectorXf &qp, float percentage)
{

    const double deg2rad = M_PI  / 180.0;

    std::vector<double> vel_limits;
    vel_limits.push_back (110.0 * deg2rad * percentage);
    vel_limits.push_back (110.0 * deg2rad * percentage);
    vel_limits.push_back (128.0 * deg2rad * percentage);
    vel_limits.push_back (128.0 * deg2rad * percentage);
    vel_limits.push_back (204.0 * deg2rad * percentage);
    vel_limits.push_back (184.0 * deg2rad * percentage);
    vel_limits.push_back (184.0 * deg2rad * percentage);

    for (unsigned i = 0; i < qp.rows(); ++i) {
        if ( std::abs(qp(i)) >= vel_limits[i] )
        {
            qp(i) = std::copysign(vel_limits[i], qp(i));
            // std::cout << "Joint Speed Limit on Joint: " << i << " set to: " << qp(i) * (1.0 / deg2rad) << std::endl;
        }
    }
}

void Kuka_LWR::saturateJointPositions( Eigen::VectorXf &q, double soft_limit )
{

    const double deg2rad = M_PI  / 180.0;

    std::vector<double> pos_limit;
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (120.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (120.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (120.0 * deg2rad) - soft_limit );
    pos_limit.push_back ( (170.0 * deg2rad) - soft_limit );



    for (unsigned i = 0; i < q.rows(); ++i) {
        if ( std::abs(q(i)) >= pos_limit[i] )
        {
            q(i) = std::copysign(pos_limit[i], q(i));
            // std::cout << "Joint Position Limit on Joint: " << i  << " set to: " << q(i) << " (rad)" << std::endl;
        }
    }
}