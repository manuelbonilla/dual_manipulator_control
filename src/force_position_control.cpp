#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

#include <std_msgs/Float64MultiArray.h>
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
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_l;
boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_l;
boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_l;
boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_l;
boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

KDL::Chain kdl_chain_l;
KDL::JntSpaceInertiaMatrix M; //Inertia matrix
KDL::JntArray C;   //Coriolis and Gravitational matrices

std::vector<double> ql, qr, xo, qpl, qpr, xpo,  qppl, qppr, xppo, f_l, f_r;
std::vector<double> tau_l, tau_r;


KDL::Frame x_l;

ros::Publisher pub_taul, pub_taur;
KDL::JntArrayAcc joint_msr_states_;
KDL::Jacobian J_l;

//TODO: document the code

void get_states(const sensor_msgs::JointState::ConstPtr msg)
{
    
    ql[0] = msg->position[0];
    ql[1] = msg->position[1];
    ql[2] = msg->position[2];
    ql[3] = msg->position[3];
    ql[4] = msg->position[4];
    ql[5] = msg->position[5];
    ql[6] = msg->position[6];
    
    std::cout << "position joint 2 l =" << ql[1] << std::endl;
    
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
    
    
    
    joint_msr_states_.q(0) = ql[0];
    joint_msr_states_.q(1) = ql[1];
    joint_msr_states_.q(2) = ql[2];
    joint_msr_states_.q(3) = ql[3];
    joint_msr_states_.q(4) = ql[4];
    joint_msr_states_.q(5) = ql[5];
    joint_msr_states_.q(6) = ql[6];
    
    fk_pos_solver_l->JntToCart(joint_msr_states_.q, x_l);
    
    id_solver_->JntToMass(joint_msr_states_.q, M);
    id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C);
    jnt_to_jac_solver_l->JntToJac(joint_msr_states_.q, J_l);
    
    std::cout << x_l << std::endl;
    
    // compute
    
    
    
}

void compute_control()
{
    
    //conti del paper
    
    tau_l[0] = 0.0;
    tau_l[1] = 0.0;
    tau_l[2] = 0.0;
    tau_l[3] = 0.0;
    tau_l[4] = 0.0;
    tau_l[5] = 0.0;
    tau_l[6] = 0.0;
    
    tau_r[0] = 0.0;
    tau_r[1] = 0.0;
    tau_r[2] = 0.0;
    tau_r[3] = 0.0;
    tau_r[4] = 0.0;
    tau_r[5] = 0.0;
    tau_r[6] = 1.0;
    
    std_msgs::Float64MultiArray msg_tau_l, msg_tau_r;
    
    msg_tau_l.data.push_back(tau_l[0]);
    msg_tau_l.data.push_back(tau_l[1]);
    msg_tau_l.data.push_back(tau_l[2]);
    msg_tau_l.data.push_back(tau_l[3]);
    msg_tau_l.data.push_back(tau_l[4]);
    msg_tau_l.data.push_back(tau_l[5]);
    msg_tau_l.data.push_back(tau_l[6]);
    
    msg_tau_r.data.push_back(tau_r[0]);
    msg_tau_r.data.push_back(tau_r[1]);
    msg_tau_r.data.push_back(tau_r[2]);
    msg_tau_r.data.push_back(tau_r[3]);
    msg_tau_r.data.push_back(tau_r[4]);
    msg_tau_r.data.push_back(tau_r[5]);
    msg_tau_r.data.push_back(tau_r[6]);
    
    pub_taul.publish(msg_tau_l);
    pub_taur.publish(msg_tau_r);
    
}

// void publish_control()
// {
//     
// }


bool init_chain(ros::NodeHandle &n)
{
    std::string robot_description, root_name, tip_name;
    
    if (!ros::param::search(n.getNamespace(),"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<n.getNamespace()<<"/robot_description)");
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
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
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
    if(!kdl_tree_.getChain("bartender_anchor", "left_arm_7_link", kdl_chain_l))
    {
        ROS_ERROR("Non si può");
        return false;
    }
    
//     if(!kdl_tree_.getChain("bartender_anchor", "right_arm_7_link", kdl_chain_r))
//     {
//         ROS_ERROR("Non si può");
//         return false;
//     }
    
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_position_control");
    
    ros::NodeHandle nh("~");
    
    ros::Subscriber sub_joint_states, sub_ft_readings_r, sub_ft_readings_l;
    
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
    
    init_chain(nh);
    
    KDL::Vector gravity_; 
    gravity_ = KDL::Vector::Zero();
    
    jnt_to_jac_solver_l.reset(new KDL::ChainJntToJacSolver(kdl_chain_l));
    fk_pos_solver_l.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_l));
    id_solver_.reset( new KDL::ChainDynParam( kdl_chain_l, gravity_) );
    M.resize(7);
    C.resize(7);
    joint_msr_states_.resize(7);
    
    
    sub_joint_states = nh.subscribe("/joint_states", 1, &get_states);
    sub_ft_readings_r = nh.subscribe("/force_torque_r", 1, &get_force_r);
    sub_ft_readings_l = nh.subscribe("/force_torque_l", 1, &get_force_l);
    
    pub_taul = nh.advertise<std_msgs::Float64MultiArray>(nh.resolveName("tau_l"),0);
    pub_taur = nh.advertise<std_msgs::Float64MultiArray>(nh.resolveName("tau_r"),0);
    
    ros::Rate rate(500);
    while(nh.ok())
    {
        compute_control();
//         publish_control();
        rate.sleep();
        ros::spinOnce();
        
    }
        
    return 0;
}
