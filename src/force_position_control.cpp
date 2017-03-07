#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/gazebo.hh>
// #include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <armadillo>


#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace Eigen;

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <math.h>
#include <pluginlib/class_list_macros.h>


using namespace Eigen;
using namespace std;

// void update(const ros::Time& time, const ros::Duration& period);
int first_step, cmd_flag_, first_time_link_states, sphere_index;
// first_step = 0;
// cmd_flag_ = 0;

void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_, bool damped = true)
{
    double lambda_ = damped ? 0.2 : 0.0;

    JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);
    JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    MatrixXd S_ = M_;   // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_l;
boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_l;
boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_l;
boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_l;
boost::scoped_ptr<KDL::ChainDynParam> id_solver_l;

boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_r;
boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_r;
boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_r;
boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_r;
boost::scoped_ptr<KDL::ChainDynParam> id_solver_r;

KDL::Chain kdl_chain_l;
KDL::JntSpaceInertiaMatrix M_l; //Inertia matrix
KDL::JntArray C_l;   //Coriolis and Gravitational matrices
KDL::JntArray G_l;
KDL::Chain kdl_chain_r;
KDL::JntSpaceInertiaMatrix M_r; //Inertia matrix
KDL::JntArray C_r;   //Coriolis and Gravitational matrices
KDL::JntArray G_r;

std::vector<double> ql, qr, qpl, qpr,  qppl, qppr, xppo, f_l, f_r;
std::vector<double> tau_l, tau_r;

Eigen::VectorXd xo(6);
Eigen::VectorXd xpo(6);

KDL::Frame x_ee_l, x_ee_l_last;
KDL::Frame x_ee_r, x_ee_r_last;
ros::Publisher pub_taul, pub_taur,  pub_command_l, pub_command_r, pub_damping_l, pub_damping_r, pub_stiffness_l, pub_stiffness_r;
KDL::JntArrayAcc joint_msr_states_l;
KDL::Jacobian J_l;
KDL::JntArrayAcc joint_msr_states_r;
KDL::Jacobian J_r;

arma::mat Ws_arma;

KDL::Frame init_pose_l, init_pose_r;

//TODO: document the code

bool flag_run_ = false;

void activate_controller_cb( const std_msgs::Float64::ConstPtr msg)
{
    if (msg->data > 0.0)
    {
        flag_run_ =  true;
        init_pose_r = x_ee_r;
        init_pose_l = x_ee_l;
        std::cout << "x_ee_l: \n" << x_ee_l << std::endl;
        std::cout << "x_ee_r: \n" << x_ee_r << std::endl;
    }
    else
    {
        flag_run_ =  false;
    }

}

void get_states(const sensor_msgs::JointState::ConstPtr msg)
{

    ql[0] = msg->position[0];
    ql[1] = msg->position[1];
    ql[2] = msg->position[2];
    ql[3] = msg->position[3];
    ql[4] = msg->position[4];
    ql[5] = msg->position[5];
    ql[6] = msg->position[6];

    qr[0] = msg->position[24];
    qr[1] = msg->position[25];
    qr[2] = msg->position[26];
    qr[3] = msg->position[27];
    qr[4] = msg->position[28];
    qr[5] = msg->position[29];
    qr[6] = msg->position[30];

    qpl[0] = msg->velocity[0];
    qpl[1] = msg->velocity[1];
    qpl[2] = msg->velocity[2];
    qpl[3] = msg->velocity[3];
    qpl[4] = msg->velocity[4];
    qpl[5] = msg->velocity[5];
    qpl[6] = msg->velocity[6];

    qpr[0] = msg->velocity[24];
    qpr[1] = msg->velocity[25];
    qpr[2] = msg->velocity[26];
    qpr[3] = msg->velocity[27];
    qpr[4] = msg->velocity[28];
    qpr[5] = msg->velocity[29];
    qpr[6] = msg->velocity[30];

    qppl[0] = msg->effort[0];
    qppl[1] = msg->effort[1];
    qppl[2] = msg->effort[2];
    qppl[3] = msg->effort[3];
    qppl[4] = msg->effort[4];
    qppl[5] = msg->effort[5];
    qppl[6] = msg->effort[6];

    qppr[0] = msg->effort[24];
    qppr[1] = msg->effort[25];
    qppr[2] = msg->effort[26];
    qppr[3] = msg->effort[27];
    qppr[4] = msg->effort[28];
    qppr[5] = msg->effort[29];
    qppr[6] = msg->effort[30];

    joint_msr_states_l.q(0) = ql[0];
    joint_msr_states_l.q(1) = ql[1];
    joint_msr_states_l.q(2) = ql[2];
    joint_msr_states_l.q(3) = ql[3];
    joint_msr_states_l.q(4) = ql[4];
    joint_msr_states_l.q(5) = ql[5];
    joint_msr_states_l.q(6) = ql[6];

    fk_pos_solver_l->JntToCart(joint_msr_states_l.q, x_ee_l);

    joint_msr_states_r.q(1) = qr[1];
    joint_msr_states_r.q(2) = qr[2];
    joint_msr_states_r.q(0) = qr[0];
    joint_msr_states_r.q(3) = qr[3];
    joint_msr_states_r.q(4) = qr[4];
    joint_msr_states_r.q(5) = qr[5];
    joint_msr_states_r.q(6) = qr[6];

    fk_pos_solver_r->JntToCart(joint_msr_states_r.q, x_ee_r);

    /* solver for dinamics*/
    id_solver_l->JntToMass(joint_msr_states_l.q, M_l);
    id_solver_l->JntToCoriolis(joint_msr_states_l.q, joint_msr_states_l.qdot, C_l);
    id_solver_l->JntToGravity(joint_msr_states_l.q, G_l);

    jnt_to_jac_solver_l->JntToJac(joint_msr_states_l.q, J_l);

    id_solver_r->JntToMass(joint_msr_states_r.q, M_r);
    id_solver_r->JntToCoriolis(joint_msr_states_r.q, joint_msr_states_r.qdot, C_r);
    jnt_to_jac_solver_r->JntToJac(joint_msr_states_r.q, J_r);
    id_solver_r->JntToGravity(joint_msr_states_r.q, G_r);
    // compute
    // flag_run_ = true;
}

void compute_control()
{
    //conti del paper

    std_msgs::Float64MultiArray msg_tau_l, msg_tau_r;
    for (int i = 0; i < 7; ++i)
    {
        if (std::abs(tau_l[i]) < 200 )
        {
            msg_tau_l.data.push_back(tau_l[i]);
        }
        else
        {
            msg_tau_l.data.push_back(0);
        }
        if (std::abs(tau_r[i]) < 200 )
        {
            msg_tau_r.data.push_back(tau_r[i]);
        }
        else
        {
            msg_tau_r.data.push_back(0);
        }
    }

    pub_taul.publish(msg_tau_l);
    pub_taur.publish(msg_tau_r);



    std_msgs::Float64MultiArray zero, q_l_feed, q_r_feed;
    for (int i = 0; i < 7; ++i)
    {
        zero.data.push_back(0.0);
        q_l_feed.data.push_back(joint_msr_states_l.q(i));
        q_r_feed.data.push_back(joint_msr_states_r.q(i));
    }

    pub_damping_l.publish(zero);
    pub_damping_r.publish(zero);
    pub_stiffness_l.publish(zero);
    pub_stiffness_r.publish(zero);
    pub_command_l.publish(q_l_feed);
    pub_command_r.publish(q_r_feed);


}

bool init_chain(ros::NodeHandle &n)
{
    std::string robot_description, root_name, tip_name;

    if (!ros::param::search(n.getNamespace(), "robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server (" << n.getNamespace() << "/robot_description)");
        return false;
    }

    // Construct an URDF model from the xml string
    std::string xml_string;

    if (n.hasParam(robot_description))
        n.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        n.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s", robot_description.c_str());
        n.shutdown();
        return false;
    }

    ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        n.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");

    KDL::Tree kdl_tree_;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        n.shutdown();
        return false;
    }

    // Populate the KDL chain
    if (!kdl_tree_.getChain("vito_anchor", "left_arm_7_link", kdl_chain_l))
    {
        ROS_ERROR("Non si puÃ²");
        return false;
    }

    if (!kdl_tree_.getChain("vito_anchor", "right_arm_7_link", kdl_chain_r))
    {
        ROS_ERROR("Non si puÃ²");
        return false;
    }

}

void get_force_l( const geometry_msgs::WrenchStamped::ConstPtr msg)
{
    f_l[0] = msg->wrench.force.x;
    f_l[1] = msg->wrench.force.y;
    f_l[2] = msg->wrench.force.z;
    f_l[3] = msg->wrench.torque.x;
    f_l[4] = msg->wrench.torque.y;
    f_l[6] = msg->wrench.torque.z;
}

void get_force_r( const geometry_msgs::WrenchStamped::ConstPtr msg)
{
    f_r[0] = msg->wrench.force.x;
    f_r[1] = msg->wrench.force.y;
    f_r[2] = msg->wrench.force.z;
    f_r[3] = msg->wrench.torque.x;
    f_r[4] = msg->wrench.torque.y;
    f_r[6] = msg->wrench.torque.z;
}

void get_obj_pos( const gazebo_msgs::LinkStates::ConstPtr msg)
{
    VectorXd qxo(4);
    // VectorXd xo(6);
    // VectorXd xpo(6);

    // ROS_INFO_STREAM("Into get_obj_pos, first_time_link_states =  " << first_time_link_states);
    if (first_time_link_states == 1)
    {
        for (int j = 0; j < msg->name.size(); ++j)
        {
            if (msg->name[sphere_index] != "cube::cube")
            {
                sphere_index++;
                // ROS_INFO_STREAM("Not Sphere Index: " << sphere_index);
            }
            else
            {
                ROS_INFO_STREAM("Cube Index: " << sphere_index);
                break;
            }
        }
        first_time_link_states = 0;
    }

    xo[0] = msg->pose[sphere_index].position.x;
    xo[1] = msg->pose[sphere_index].position.y;
    xo[2] = msg->pose[sphere_index].position.z;

    qxo[0] = msg->pose[sphere_index].orientation.x;
    qxo[1] = msg->pose[sphere_index].orientation.y;
    qxo[2] = msg->pose[sphere_index].orientation.z;
    qxo[3] = msg->pose[sphere_index].orientation.w;

    // if ((2 * (qxo[0]*qxo[1] + qxo[2]*qxo[3])) >= 0) {
    xo[3] = atan(2 * (qxo[0] * qxo[1] + qxo[2] * qxo[3]) / (1 - 2 * (qxo[1] * qxo[1] + qxo[2] * qxo[2])));
    // } else {
    //     xo[3] = atan(2 * (qxo[0] * qxo[1] + qxo[2] * qxo[3]) / (1 - 2 * (qxo[1] * qxo[1] + qxo[2] * qxo[2]))) + 3.14;
    // }

    xo[4] = asin(2 * (qxo[0] * qxo[2] - qxo[3] * qxo[1]));

    // if ((2 * (qxo[0]*qxo[3] + qxo[1]*qxo[2])) >= 0) {
    xo[5] = atan(2 * (qxo[0] * qxo[3] + qxo[1] * qxo[2]) / (1 - 2 * (qxo[2] * qxo[2] + qxo[3] * qxo[3])));
    // } else {
    //     xo[5] = atan(2 * (qxo[0] * qxo[3] + qxo[1] * qxo[2]) / (1 - 2 * (qxo[2] * qxo[2] + qxo[3] * qxo[3]))) + 3.14;
    // }

    xpo(0) = msg->twist[sphere_index].linear.x;
    xpo(1) = msg->twist[sphere_index].linear.y;
    xpo(2) = msg->twist[sphere_index].linear.z;
    xpo(3) = msg->twist[sphere_index].angular.x;
    xpo(4) = msg->twist[sphere_index].angular.y;
    xpo(5) = msg->twist[sphere_index].angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_position_control");

    ros::NodeHandle nh("~");

    ros::Subscriber sub_joint_states, sub_ft_readings_r, sub_ft_readings_l, sub_get_obj_pos;

    ql.resize(7);
    qr.resize(7);
    qpl.resize(7);
    qpr.resize(7);
    qppl.resize(7);
    qppr.resize(7);
    xo.resize(6);
    xpo.resize(6);
    xppo.resize(6);
    tau_l.resize(7);
    tau_r.resize(7);
    f_l.resize(6);
    f_r.resize(6);

    sphere_index = 0;
    first_time_link_states = 1;

    sub_joint_states = nh.subscribe("/joint_states", 1, &get_states);
    sub_ft_readings_r = nh.subscribe("/force_torque_r", 1, &get_force_r);
    sub_ft_readings_l = nh.subscribe("/force_torque_l", 1, &get_force_l);
    sub_get_obj_pos = nh.subscribe("/gazebo/link_states", 1, &get_obj_pos);
    ros::Subscriber activate_controller_sub = nh.subscribe("/activate_controller", 1, &activate_controller_cb);

    pub_taul = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/additional_torque", 0);
    pub_taur = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/additional_torque", 0);
    pub_command_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/command", 0);
    pub_command_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/command", 0);
    pub_damping_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/damping", 0);
    pub_damping_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/damping", 0);
    pub_stiffness_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/stiffness", 0);
    pub_stiffness_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/stiffness", 0);

    KDL::Vector gravity_;
    gravity_ = KDL::Vector::Zero();

    init_chain(nh);

    jnt_to_jac_solver_l.reset(new KDL::ChainJntToJacSolver(kdl_chain_l));
    fk_pos_solver_l.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_l));
    id_solver_l.reset(new KDL::ChainDynParam(kdl_chain_l, gravity_) );
    M_l.resize(7);
    C_l.resize(7);
    G_l.resize(7);
    joint_msr_states_l.resize(7);

    jnt_to_jac_solver_r.reset(new KDL::ChainJntToJacSolver(kdl_chain_r));
    fk_pos_solver_r.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_r));
    id_solver_r.reset(new KDL::ChainDynParam(kdl_chain_r, gravity_) );
    M_r.resize(7);
    C_r.resize(7);
    G_r.resize(7);
    joint_msr_states_r.resize(7);

    //
    J_l.resize(kdl_chain_l.getNrOfJoints());
    J_r.resize(kdl_chain_r.getNrOfJoints());


    sleep(1);
    while (!flag_run_)
    {
        // std::cout << "wait\n";
        ros::spinOnce();
    }

    //----------------------------------------------------------------------------------------------------------------
    // Calcoli per braccio SX

    ros::Time start, finish;
    float dt = 1.0 / 500.0;
    ros::Rate rate(500);


    int ccc = 0;
    double t_total = 0.0;
    while (ros::ok() && flag_run_)
    {
        ros::spinOnce();
        start = ros::Time::now();
        t_total += dt;

        VectorXd G(14);

        G << G_l(0), G_l(1), G_l(2), G_l(3), G_l(4), G_l(5), G_l(6), G_r(0), G_r(1), G_r(2), G_r(3), G_r(4), G_r(5), G_r(6);

        //devo riempire delle matrici e vettori di Eigen per fare le operazioni //

        //dinamica aumentata del manipolatore a due braccia//

        VectorXd q(14); //vettore aumentato delle variabili di giunto//

        q << ql[0] , ql[1] , ql[2] , ql[3] , ql[4] , ql[5], ql[6] , qr[0] , qr[1] , qr[2] , qr[3] , qr[4] , qr[5] , qr[6] ;

        VectorXd qp(14); //vettore aumentato delle velocità angolari di giunto//

        qp << qpl[0] , qpl[1] , qpl[2] , qpl[3] , qpl[4] , qpl[5], qpl[6] , qpr[0] , qpr[1] , qpr[2] , qpr[3] , qpr[4] , qpr[5] , qpr[6] ;

        VectorXd qpp(14); //vettore aumentato delle accelerazioni angolari di giunto//

        qpp << qppl[0] , qppl[1] , qppl[2] , qppl[3] , qppl[4] , qppl[5], qppl[6] , qppr[0] , qppr[1] , qppr[2] , qppr[3] , qppr[4] , qppr[5] , qppr[6] ;

        MatrixXd Jleft(6, 7);
        MatrixXd Jright(6, 7);

        for (int i = 0 ; i < 6 ; i++) {
            for (int j = 0 ; j < 7 ; j++ ) {
                Jleft(i, j) = J_l(i, j);
                Jright(i, j) = J_r(i, j);
            }
        }

        MatrixXd J(12, 14); //jacobiano analitico aumentato//

        J.topLeftCorner(6, 7) = Jleft;
        J.topRightCorner(6, 7) = MatrixXd::Zero(6, 7);
        J.bottomLeftCorner(6, 7) = MatrixXd::Zero(6, 7);
        J.bottomRightCorner(6, 7) = Jright;

        MatrixXd Mleft(7, 7);
        MatrixXd Mright(7, 7);

        for (int i = 0 ; i < 7 ; i++) {
            for (int j = 0 ; j < 7 ; j++) {
                Mleft(i, j) = M_l(i, j);
                Mright(i, j) = M_r(i, j);
            }
        }

        MatrixXd M(14, 14); //matrice delle inerzie aumentata//

        M.topLeftCorner(7, 7) = Mleft;
        M.topRightCorner(7, 7) = MatrixXd::Zero(7, 7);
        M.bottomLeftCorner(7, 7) = MatrixXd::Zero(7, 7);
        M.bottomRightCorner(7, 7) = Mright;

        VectorXd C(14); //contributi di coriolis + centrifugi augmented//

        C << C_l(0) , C_l(1) , C_l(2) , C_l(3) , C_l(4) , C_l(5) , C_l(6) , C_r(0) , C_r(1) , C_r(2), C_r(3) , C_r(4) , C_r(5) , C_r(6) ;

        //dinamica e cinematica dell'oggetto //

        VectorXd r(6);//posizione e orientazione oggetto//

        r << xo(0) , xo(1) , xo(2) , xo(3) , xo(4) , xo(5);

        VectorXd rp(6);

        rp = xpo;

        double massa;
        massa = 1;

        MatrixXd tensor(3, 3);

        tensor << 0.25 , 0 , 0 ,
               0 , 0.25 , 0 ,
               0 , 0 , 0.25 ;

        MatrixXd I3(3, 3);

        I3 = MatrixXd::Identity(3, 3);

        MatrixXd Ib(6, 6); //matrice delle inerzie per oggetto//

        Ib.topLeftCorner(3, 3) = massa * I3;
        Ib.topRightCorner(3, 3) = MatrixXd::Zero(3, 3);
        Ib.bottomLeftCorner(3, 3) = MatrixXd::Zero(3, 3);
        Ib.bottomRightCorner(3, 3) = tensor;

        VectorXd Qb(6);
        VectorXd w(3);
        VectorXd Iw(3);
        VectorXd Cross(3);//contributi di Coriolis sull'oggetto// sarebbe w x (Iw)

        w << rp(3) , rp(4) , rp(5);
        Iw << tensor(0, 0)*w(0) - tensor(0, 2)*w(2) , tensor(1, 1)*w(1) , tensor(2, 2)*w(2) - tensor(2, 0)*w(0) ;
        Cross << w(1)*Iw(2) - Iw(1)*w(2) , -w(0)*Iw(2) + Iw(0)*w(2) , w(0)*Iw(1) - Iw(0)*w(1) ;

        Qb << 0 , 0 , 0/*-massa * 9.81*/ , Cross(0) , Cross(1) , Cross(2) ;

        MatrixXd Tr(3, 3);

        Tr << 0 , -sin(r[3]) , cos(r[3])*sin(r[4]) ,
        0 , cos(r[3]) , sin(r[3])*sin(r[4]) ,
        1 , 0 , cos(r[4]) ;//jacobiano di trasformazione//

        MatrixXd T(6, 6); //trasformazione Velocità-Twist//

        T.topLeftCorner(3, 3) = I3;
        T.topRightCorner(3, 3) = MatrixXd::Zero(3, 3);
        T.bottomLeftCorner(3, 3) = MatrixXd::Zero(3, 3);
        T.bottomRightCorner(3, 3) = Tr;

        MatrixXd Tp(6, 6);
        MatrixXd Tlast(6, 6);

        if (first_step) {
            Tlast = T;
            first_step = 0;
        }

        Tp = (T - Tlast) / dt;

        VectorXd Qr(6);

        Qr = Ib * Tp * rp + Qb ;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //mi servono le posizioni(e orientazioni) cartesiane dei 2 end effectors

        VectorXd p1(3);
        VectorXd p2(3);

        p1 << x_ee_l.p(0) , x_ee_l.p(1) , x_ee_l.p(2);
        p2 << x_ee_r.p(0) , x_ee_r.p(1) , x_ee_r.p(2);

        MatrixXd P1(3, 3);
        MatrixXd P2(3, 3);

        P1 << 0 , -p1(2) , p1(1) ,
        p1(2) , 0 , -p1(0) ,
        -p1(1) , p1(0) , 0 ;

        P2 << 0 , -p2(2) , p2(1) ,
        p2(2) , 0 , -p2(0) ,
        -p2(1) , p2(0) , 0 ;

        //vincoli cinematici e di forza tra oggetto e manipolatore//

        MatrixXd Rc1(3, 3); //matrice di rotazione che mi porta nel primo punto di contatto//

        Rc1 <<  x_ee_l.M.data[0], x_ee_l.M.data[1], x_ee_l.M.data[2],
            x_ee_l.M.data[3], x_ee_l.M.data[4], x_ee_l.M.data[5],
            x_ee_l.M.data[6], x_ee_l.M.data[7], x_ee_l.M.data[8];

        MatrixXd Rc2(3, 3); //matrice di rotazione che mi porta nel primo punto di contatto//

        Rc2 <<  x_ee_r.M.data[0], x_ee_r.M.data[1], x_ee_r.M.data[2],
            x_ee_r.M.data[3], x_ee_r.M.data[4], x_ee_r.M.data[5],
            x_ee_r.M.data[6], x_ee_r.M.data[7], x_ee_r.M.data[8];

        MatrixXd RC1(6, 6);
        MatrixXd RC2(6, 6);

        RC1.topLeftCorner(3, 3) = Rc1;
        RC1.topRightCorner(3, 3) = MatrixXd::Zero(3, 3);
        RC1.bottomLeftCorner(3, 3) = MatrixXd::Zero(3, 3);
        RC1.bottomRightCorner(3, 3) = Rc1;

        RC2.topLeftCorner(3, 3) = Rc2;
        RC2.topRightCorner(3, 3) = MatrixXd::Zero(3, 3);
        RC2.bottomLeftCorner(3, 3) = MatrixXd::Zero(3, 3);
        RC2.bottomRightCorner(3, 3) = Rc2;

        MatrixXd Sc1(6, 3);
        MatrixXd Sc2(6, 3);

        Sc1 <<  1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;

        Sc2 <<  1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;

        MatrixXd S1(6, 3);
        MatrixXd S2(6, 3);

        S1 = RC1 * Sc1;
        S2 = RC2 * Sc2;

        MatrixXd S(12, 6);

        S.topLeftCorner(6, 3) = S1;
        S.topRightCorner(6, 3) = MatrixXd::Zero(6, 3);
        S.bottomLeftCorner(6, 3) = MatrixXd::Zero(6, 3);
        S.bottomRightCorner(6, 3) = S2;

        MatrixXd W(6, 12);
        MatrixXd W1(6, 6);
        MatrixXd W2(6, 6);

        W1.topRightCorner(3, 3) = I3;
        W1.topLeftCorner(3, 3) = MatrixXd::Zero(3, 3);
        W1.bottomLeftCorner(3, 3) = P1;
        W1.bottomRightCorner(3, 3) = I3;

        W2.topRightCorner(3, 3) = I3;
        W2.topLeftCorner(3, 3) = MatrixXd::Zero(3, 3);
        W2.bottomLeftCorner(3, 3) = P2;
        W2.bottomRightCorner(3, 3) = I3;

        W.block(0, 0, 6, 6) = W1;
        W.block(0, 5, 6, 6) = W2;

        ///////////////////////// W = W.transpose();//perchè non so se me la giustapposizione avviene in orizzontale o in verticale//

        MatrixXd Ws(6, 6);

        Ws = W * S;



        //per noi non ci sono vincoli ambientali quindi nell'articolo Ef = 0 ed E = Ep //

        MatrixXd E(6, 6);

        E << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1; //per ora non so com'è fatta ????????//

        MatrixXd Elast(6, 6);
        MatrixXd Ep(6, 6);

        if (first_step) {
            Elast = E;
            first_step = 0;
        }

        Ep = (E - Elast) / dt;

        MatrixXd Einv(6, 6);

        pseudo_inverse(E, Einv);

        //devo scrivere un po di matrici trasposte,derivate e pseudoinverse //

        //////////////////////////////////////////////////////

        MatrixXd St(6, 12);

        St = S.transpose();

        MatrixXd Stpinv(12, 6);

        pseudo_inverse(St, Stpinv);

        MatrixXd S_pinv(6, 12);

        pseudo_inverse(S, S_pinv);

        MatrixXd Stp(6, 12);
        MatrixXd St_last(6, 12);

        if (first_step) {
            St_last = St;
            first_step = 0;
        }

        Stp = (St - St_last) / dt; //S trasposta e derivata//

        ///////////////////////////////////////////////////////////

        MatrixXd Jpinv(14, 12);

        pseudo_inverse(J, Jpinv);

        MatrixXd Jp(12, 14);
        MatrixXd Jlast(12, 14);

        if (first_step) {
            Jlast = J;
            first_step = 0;
        }

        Jp = (J - Jlast) / dt;

        /////////////////////////////////////////////////////////7

        MatrixXd Wst(6, 6);

        Wst = Ws.transpose();

        MatrixXd Wstp(6, 6);

        MatrixXd Wstlast(6, 6);

        if (first_step) {
            Wstlast = Wst;
            first_step = 0;
        }

        Wstp = (Wst - Wstlast) / dt;

        //////////////////////////////////////////////

        //ora scrivo le due parti della legge di controllo//

        VectorXd Tau_A(14);
        VectorXd Tau_E(14);
        VectorXd Tau_C(14);

        VectorXd U1(6);
        MatrixXd Kv(6, 6);
        MatrixXd Kp(6, 6);
        MatrixXd I6(6, 6);

        I6 = MatrixXd::Identity(6, 6);

        double kv;
        double kp;

        kv = 0.00001;
        kp = 10;

        Kv = kv * I6;
        Kp = kp * I6;

        VectorXd rd(6);
        VectorXd rpd(6);
        VectorXd rppd(6);

        rd << -0.6, 0, 0.2, 0, 0, 0;
        rpd << 0.0, 0.0, 0.0, 0, 0, 0;
        rppd << 0, 0, 0, 0, 0, 0;

        U1 = rppd + Kv * (rpd - rp) + Kp * (rd - r);
        VectorXd rppdes(6);

        rppdes = Einv * (U1 - Ep * rp);

        VectorXd qppdes(14);
        VectorXd d1(12);
        VectorXd d2(14);
        // VectorXd d3(6);

        d1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        d2 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; //per ora senza elementi del Kernel dello spazio cinematico e vincolato
        // d3 << 0, 0, 0, 0, 0, 0;

        MatrixXd I12(12, 12);
        MatrixXd I14(14, 14);

        I12 = MatrixXd::Identity(12, 12);
        I14 = MatrixXd::Identity(14, 14);

        qppdes = Jpinv * (Stpinv * (Wst * T * rppdes + (Wstp * T + Wst * Tp) * rp - Stp * J * qp) + (I12 - Stpinv * St) * d1 - Jp * qp);// + (I14 - Jpinv * J) * d2 ;

        Eigen::VectorXd ep(12);
        // ep = Eigen::MatrixXd::Zero(12,1);

        VectorXd xee_l(6);
        VectorXd xee_r(6);

        // std::cout << "Rc1 \n" << Rc1(1);
        xee_l << x_ee_l.p(0) , x_ee_l.p(1) , x_ee_l.p(2) , Rc1(1) , Rc1(2) , Rc1(3);
        xee_r << x_ee_r.p(0) , x_ee_r.p(1) , x_ee_r.p(2) , Rc2(1) , Rc2(2) , Rc2(3);

        // for (int i=0;i<7;++i){
        //         xee_l(i) = x_ee_l.p(i);
        // }

        // for (int i=0;i<7;++i){
        //         xee_r(i) = x_ee_r.p(i);
        // }

        VectorXd xpee_l(6);
        VectorXd xpee_l_last(6);

        if (first_step) {
            xpee_l_last = xee_l;
            first_step = 0;
            x_ee_l_last = x_ee_l;
        }

        xpee_l = (xee_l - xpee_l_last) / dt;

        VectorXd xpee_r(6);
        VectorXd xpee_r_last(6);

        if (first_step) {
            xpee_r_last = xee_r;
            first_step = 0;
            x_ee_r_last = x_ee_r;
        }

        xpee_r = (xee_r - xpee_r_last) / dt;

        init_pose_l.p = init_pose_l.p * .1 * sin(2.0 * M_PI * 1.0 / 10.0 * t_total);
        init_pose_r.p = init_pose_r.p * .1 * sin(2.0 * M_PI * 1.0 / 10.0 * t_total);
        KDL::Twist pose_error_l = diff(x_ee_l, init_pose_l);
        KDL::Twist pose_error_r = diff(x_ee_r, init_pose_r);

        KDL::Twist pose_error_derivative_l = diff(x_ee_l, x_ee_l_last) / dt;
        KDL::Twist pose_error_derivative_r = diff(x_ee_r, x_ee_r_last) / dt;

        double kp_control;
        nh.param<double>("kp", kp_control , 1.0);
        double kv_control;
        nh.param<double>("kv", kv_control , 0.2);
        for (int i = 0; i < 6; ++i)
        {
            // if (i<3){
            ep(i) = /*(rd(i) - x_ee_l.p(i));*/ kv_control* pose_error_derivative_l(i) + kp_control* pose_error_l(i);
            ep(i + 6) = /*(rd(i) - x_ee_r.p(i));*/ kv_control * pose_error_derivative_r(i) + kp_control* pose_error_r(i);
            // }
            // else
            // {
            // ep(i) = 0;
            // ep(i+6) = 0;
            // }
        }
        // ep.block(6,0,6,1) = (rd - r);

        qppdes = Jpinv * ep;
        // qppdes = Eigen::MatrixXd::Zero(12,1);

        // qppdes<< 1,1,1,1,1,1,1,1,1,1,1,1,1,1;
        // VectorXd Dq(14);
        VectorXd qd(14);
        qd << 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15;
        // for(int i=0; i<14;++i)
        // {
        //     qppdes(i) = 1000.0*(qd(i) - q(i)) ;
        // }


        Tau_A = M * qppdes + C /*+ G*/ ;

        VectorXd Fhsd(6);
        MatrixXd Wspinv(6, 6);
        MatrixXd Proj(6, 6);
        // MatrixXd I6(6,6);

        I6 = MatrixXd::Identity(6, 6);

        pseudo_inverse(Ws, Wspinv);

        // Fhsd = Wspinv * (Qr);

        MatrixXd Jt(14, 12);
        Jt = J.transpose();

        arma::mat Wspinv_arma(6, 6);
        for (int i = 0 ; i < 6 ; ++i)
        {
            for (int j = 0 ; j < 6 ; ++j)
            {
                Wspinv_arma(i, j) = Wspinv(i, j);
            }

        }

        arma::mat null_Wspinv_arma = arma::null(Wspinv_arma);

        // int nc = null_Wspinv_arma.n_rows;
        Eigen::MatrixXd null_Wspinv(null_Wspinv_arma.n_rows, null_Wspinv_arma.n_cols);

        for (int i = 0 ; i < null_Wspinv_arma.n_rows ; ++i)
        {
            for (int j = 0 ; j < null_Wspinv_arma.n_cols ; ++j)
            {
                null_Wspinv(i, j) = null_Wspinv_arma(i, j);
            }

        }

        VectorXd d3(null_Wspinv_arma.n_cols);
        for (int i = 0 ; i < null_Wspinv_arma.n_cols ; ++i)
        {
            d3(i) = 1;
        }

        Fhsd = Wspinv * (Ib * T * rppdes + Qr) + null_Wspinv * d3;

        Tau_E = Jt * S * Fhsd;

        // std::cout << "te\n" << Tau_E << std::endl;


        Tau_C = Tau_A ;//+ Tau_E;

        ////////////////////////////////////////////////

        for (int i = 0 ; i < 7 ; i++) {
            // if (Tau_C(i) < 20) {
            tau_l[i] = Tau_C(i);
            // }
            // else {
            // tau_l[i] = 20;
            // }
        }

        double j;

        for (int i = 7 ; i < 14 ; i++) {
            j = i - 7;
            // if (Tau_C(i) < 20) {

            tau_r[j] = Tau_C(i);
            // }
            // else {
            // tau_r[i] = 20;
            // }
        }

//-------------------------------------------------------------------------//
        // publish_control();

        compute_control();

        rate.sleep();
        ros::spinOnce();
        finish = ros::Time::now();
        // dt = finish.toSec() - start.toSec();

        ccc++;
    } // end while

    return 0;
} // end main