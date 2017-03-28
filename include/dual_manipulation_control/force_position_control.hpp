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

#include <iostream>
#include <cmath>

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

std::vector<float> ql, qr, qpl, qpr,  qppl, qppr, xppo, f_l, f_r;
std::vector<float> tau_l, tau_r;

Eigen::VectorXf xo;
Eigen::VectorXf xpo;
float t_total;

int n_joints, n_dim_space;
KDL::Frame x_ee_l, x_ee_l_last;
KDL::Frame x_ee_r, x_ee_r_last;
ros::Publisher pub_taul, pub_taur,  pub_command_l, pub_command_r, pub_damping_l, pub_damping_r, pub_stiffness_l, pub_stiffness_r, pub_errors;
KDL::JntArrayAcc joint_msr_states_l;
KDL::Jacobian J_l;
KDL::JntArrayAcc joint_msr_states_r;
KDL::Jacobian J_r;

arma::mat Ws_arma;

KDL::Frame init_pose_l, init_pose_r;

bool first_time_joint_states;
std::vector<int> joint_index;
std::vector<float> tau_max;

KDL::Twist pose_error_l;
KDL::Twist pose_error_r;

//TODO: document the code

bool flag_run_ = false;


void get_joint_index(const sensor_msgs::JointState::ConstPtr msg)
{
	std::vector<std::string> angle_names = {"left_arm_a1_joint", "left_arm_a2_joint", "left_arm_e1_joint", "left_arm_a3_joint", "left_arm_a4_joint", "left_arm_a5_joint", "left_arm_a6_joint",
	                                        "right_arm_a1_joint", "right_arm_a2_joint", "right_arm_e1_joint", "right_arm_a3_joint", "right_arm_a4_joint", "right_arm_a5_joint", "right_arm_a6_joint"
	                                       };
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
	for (int i = 0; i < angle_names.size(); ++i)
		std::cout << "joint " << angle_names[i] << "in on joint_states index: " << joint_index[i] << std::endl;
	std::cout << std::endl;
}

void get_states(const sensor_msgs::JointState::ConstPtr msg)
{

	if (first_time_joint_states)
	{
		get_joint_index(msg);
		first_time_joint_states = false;
	}

	for (int i = 0; i < n_joints; i++)
	{
		ql[i] = msg->position[joint_index[i]];
		qr[i] = msg->position[joint_index[i + n_joints]];
		qpl[i] = msg->velocity[joint_index[i]];
		qpr[i] = msg->velocity[joint_index[i + n_joints]];
		qppl[i] = msg->effort[joint_index[i]] * 0.;
		qppr[i] = msg->effort[joint_index[i + n_joints]] * 0.;
		joint_msr_states_l.q(i) = ql[i];
		joint_msr_states_r.q(i) = qr[i];
		joint_msr_states_l.qdot(i) = qpl[i];
		joint_msr_states_r.qdot(i) = qpr[i];
	}


	fk_pos_solver_l->JntToCart(joint_msr_states_l.q, x_ee_l);
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

void get_statesr(const sensor_msgs::JointState::ConstPtr msg)
{

	std::vector<float> index = {0, 1, 6, 2, 3, 4, 5};
	for (int i = 0; i < n_joints; i++)
	{

		qr[i] = msg->position[index[i]];
		qpr[i] = msg->velocity[index[i]];
		qppr[i] = msg->effort[index[i]] * 0.;
		joint_msr_states_r.q(i) = qr[i];
		joint_msr_states_r.qdot(i) = qpr[i];
	}


	fk_pos_solver_r->JntToCart(joint_msr_states_r.q, x_ee_r);

	/* solver for dinamics*/

	id_solver_r->JntToMass(joint_msr_states_r.q, M_r);
	id_solver_r->JntToCoriolis(joint_msr_states_r.q, joint_msr_states_r.qdot, C_r);
	jnt_to_jac_solver_r->JntToJac(joint_msr_states_r.q, J_r);
	id_solver_r->JntToGravity(joint_msr_states_r.q, G_r);
	// compute
	// flag_run_ = true;
}


void get_statesl(const sensor_msgs::JointState::ConstPtr msg)
{

	std::vector<float> index = {0, 1, 6, 2, 3, 4, 5};
	for (int i = 0; i < n_joints; i++)
	{

		ql[i] = msg->position[index[i]];
		qpl[i] = msg->velocity[index[i]];
		qppl[i] = msg->effort[index[i]] * 0.;
		joint_msr_states_l.q(i) = ql[i];
		joint_msr_states_l.qdot(i) = qpl[i];
	}


	fk_pos_solver_l->JntToCart(joint_msr_states_l.q, x_ee_l);

	/* solver for dinamics*/

	id_solver_l->JntToMass(joint_msr_states_l.q, M_l);
	id_solver_l->JntToCoriolis(joint_msr_states_l.q, joint_msr_states_l.qdot, C_l);
	jnt_to_jac_solver_l->JntToJac(joint_msr_states_l.q, J_l);
	id_solver_l->JntToGravity(joint_msr_states_l.q, G_l);
	// compute
	// flag_run_ = true;
}

void compute_control(bool publish_all)
{
	//conti del paper
	float tau_scale(0.8);

	std_msgs::Float64MultiArray msg_tau_l, msg_tau_r;
	for (int i = 0; i < n_joints; ++i)
	{
		if (std::abs(tau_l[i]) < tau_max[i]*tau_scale )
		{
			msg_tau_l.data.push_back(tau_l[i]);
		}
		else
		{
			// float tau_sat = tau_max[i] * tau_scale;
			// std::copysign(tau_sat, tau_l[i]);
			if (tau_l[i] < 0)
				msg_tau_l.data.push_back(-tau_max[i] * tau_scale);
			else
				msg_tau_l.data.push_back(tau_max[i] * tau_scale);
		}
		if (std::abs(tau_r[i]) < tau_max[i]*tau_scale )
		{
			msg_tau_r.data.push_back(tau_r[i]);
		}
		else
		{
			if (tau_r[i] < 0)
				msg_tau_r.data.push_back(-tau_max[i] * tau_scale);
			else
				msg_tau_r.data.push_back(tau_max[i] * tau_scale);
		}
	}

	std_msgs::Float64MultiArray zero, q_l_feed, q_r_feed, msg_errors;

	for (int i = 0; i < n_dim_space * 2; ++i)
	{
		if (i < n_dim_space)
			msg_errors.data.push_back(pose_error_l(i));
		else
			msg_errors.data.push_back(pose_error_r(i - n_dim_space));
	}

	for (int i = 0; i < n_joints; ++i)
	{
		zero.data.push_back(0.0);
		q_l_feed.data.push_back(joint_msr_states_l.q(i));
		q_r_feed.data.push_back(joint_msr_states_r.q(i));
	}

	if (publish_all)
	{

		pub_damping_l.publish(zero);
		pub_damping_r.publish(zero);
		pub_stiffness_l.publish(zero);
		pub_stiffness_r.publish(zero);

	}

	pub_command_l.publish(q_l_feed);
	pub_command_r.publish(q_r_feed);
	pub_errors.publish(msg_errors);
	pub_taul.publish(msg_tau_l);
	pub_taur.publish(msg_tau_r);

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

	return true;

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


bool init_variables(ros::NodeHandle &nh)
{
	n_joints = 7;
	n_dim_space = 6;
	ql.resize(n_joints);
	qr.resize(n_joints);
	qpl.resize(n_joints);
	qpr.resize(n_joints);
	qppl.resize(n_joints);
	qppr.resize(n_joints);
	xo.resize(n_dim_space);
	xpo.resize(n_dim_space);
	xppo.resize(n_dim_space);
	tau_l.resize(n_joints);
	tau_r.resize(n_joints);
	f_l.resize(n_dim_space);
	f_r.resize(n_dim_space);

	first_time_joint_states = true;

	sphere_index = 0;
	first_time_link_states = 1;

	KDL::Vector gravity_;
	gravity_ = KDL::Vector::Zero();

	if (!init_chain(nh))
	{
		std::cout << "Not posible to initialize chains" << std::endl;
		return false;
	}

	jnt_to_jac_solver_l.reset(new KDL::ChainJntToJacSolver(kdl_chain_l));
	fk_pos_solver_l.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_l));
	id_solver_l.reset(new KDL::ChainDynParam(kdl_chain_l, gravity_) );
	M_l.resize(n_joints);
	C_l.resize(n_joints);
	G_l.resize(n_joints);
	joint_msr_states_l.resize(n_joints);

	jnt_to_jac_solver_r.reset(new KDL::ChainJntToJacSolver(kdl_chain_r));
	fk_pos_solver_r.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_r));
	id_solver_r.reset(new KDL::ChainDynParam(kdl_chain_r, gravity_) );
	M_r.resize(n_joints);
	C_r.resize(n_joints);
	G_r.resize(n_joints);
	joint_msr_states_r.resize(n_joints);

	//
	J_l.resize(kdl_chain_l.getNrOfJoints());
	J_r.resize(kdl_chain_r.getNrOfJoints());

	tau_max = std::vector<float>({176.0, 176.0, 100.0, 100.0, 100.0, 38.0, 38.0});
	return true;
}

