#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

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

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <iostream>
#include <math.h>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <boost/scoped_ptr.hpp>
#include <armadillo>

int first_step, cmd_flag_, first_time_link_states, sphere_index;

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

bool first_time_joint_states;
std::vector<int> joint_index;

//TODO: document the code

bool flag_run_ = false;

void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_, bool damped = true)
{
	double lambda_ = damped ? 0.02 : 0.0;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	Eigen::MatrixXd S_ = M_;   // copying the dimensions of M_, its content is not needed.
	S_.setZero();

	for (int i = 0; i < sing_vals_.size(); i++)
		S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

	M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

void get_joint_index(const sensor_msgs::JointState::ConstPtr msg)
{
	std::vector<std::string> angle_names = {"left_arm_a1_joint", "left_arm_a2_joint", "left_arm_e1_joint", "left_arm_a3_joint", "left_arm_a4_joint", "left_arm_a5_joint", "left_arm_a6_joint",
	                                        "right_arm_a1_joint", "right_arm_a2_joint", "right_arm_e1_joint", "right_arm_a3_joint", "right_arm_a4_joint", "right_arm_a5_joint", "right_arm_a6_joint"};
	for (auto js : angle_names)
	{
		int i = 0;
		for (auto msg_js : msg->name)
		{
			if (msg_js == js)
			{
				joint_index.push_back(i);
			}
			i++;
		}
	}
}

void get_states(const sensor_msgs::JointState::ConstPtr msg)
{

	if (first_time_joint_states)
	{
		get_joint_index(msg);
		first_time_joint_states = false;
	}
	ql[0] = msg->position[joint_index[0]];
	ql[1] = msg->position[joint_index[1]];
	ql[2] = msg->position[joint_index[2]];
	ql[3] = msg->position[joint_index[3]];
	ql[4] = msg->position[joint_index[4]];
	ql[5] = msg->position[joint_index[5]];
	ql[6] = msg->position[joint_index[6]];

	qr[0] = msg->position[joint_index[7]];
	qr[1] = msg->position[joint_index[8]];
	qr[2] = msg->position[joint_index[9]];
	qr[3] = msg->position[joint_index[10]];
	qr[4] = msg->position[joint_index[11]];
	qr[5] = msg->position[joint_index[12]];
	qr[6] = msg->position[joint_index[13]];

	qpl[0] = msg->velocity[joint_index[0]];
	qpl[1] = msg->velocity[joint_index[1]];
	qpl[2] = msg->velocity[joint_index[2]];
	qpl[3] = msg->velocity[joint_index[3]];
	qpl[4] = msg->velocity[joint_index[4]];
	qpl[5] = msg->velocity[joint_index[5]];
	qpl[6] = msg->velocity[joint_index[6]];

	qpr[0] = msg->velocity[joint_index[7]];
	qpr[1] = msg->velocity[joint_index[8]];
	qpr[2] = msg->velocity[joint_index[9]];
	qpr[3] = msg->velocity[joint_index[10]];
	qpr[4] = msg->velocity[joint_index[11]];
	qpr[5] = msg->velocity[joint_index[12]];
	qpr[6] = msg->velocity[joint_index[13]];

	qppl[0] = msg->effort[joint_index[0]];
	qppl[1] = msg->effort[joint_index[1]];
	qppl[2] = msg->effort[joint_index[2]];
	qppl[3] = msg->effort[joint_index[3]];
	qppl[4] = msg->effort[joint_index[4]];
	qppl[5] = msg->effort[joint_index[5]];
	qppl[6] = msg->effort[joint_index[6]];

	qppr[0] = msg->effort[joint_index[7]];
	qppr[1] = msg->effort[joint_index[8]];
	qppr[2] = msg->effort[joint_index[9]];
	qppr[3] = msg->effort[joint_index[10]];
	qppr[4] = msg->effort[joint_index[11]];
	qppr[5] = msg->effort[joint_index[12]];
	qppr[6] = msg->effort[joint_index[13]];

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
	Eigen::VectorXd qxo(4);
	// Eigen::VectorXd xo(6);
	// Eigen::VectorXd xpo(6);

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
	xo[2] = msg->pose[sphere_index].position.z - 1.0;

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
	xpo(2) = msg->twist[sphere_index].linear.z;;
	xpo(3) = msg->twist[sphere_index].angular.x;
	xpo(4) = msg->twist[sphere_index].angular.y;
	xpo(5) = msg->twist[sphere_index].angular.z;
}


void init_variables(ros::NodeHandle &nh)
{
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

	first_time_joint_states = true;

	sphere_index = 0;
	first_time_link_states = 1;

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
}

