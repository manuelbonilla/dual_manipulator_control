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

Eigen::VectorXd xo;
Eigen::VectorXd xpo;

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

KDL::Twist pose_error_l;
KDL::Twist pose_error_r;

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

void compute_control()
{
	//conti del paper

	std_msgs::Float64MultiArray msg_tau_l, msg_tau_r;
	for (int i = 0; i < n_joints; ++i)
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



	std_msgs::Float64MultiArray zero, q_l_feed, q_r_feed, msg_errors;
	for (int i = 0; i < n_joints; ++i)
	{
		zero.data.push_back(0.0);
		q_l_feed.data.push_back(joint_msr_states_l.q(i));
		q_r_feed.data.push_back(joint_msr_states_r.q(i));
	}
	for (int i = 0; i < n_dim_space*2; ++i)
	{
		if (i < n_dim_space)
			msg_errors.data.push_back(pose_error_l(i));
		else
			msg_errors.data.push_back(pose_error_r(i - 6));
	}


	pub_damping_l.publish(zero);
	pub_damping_r.publish(zero);
	pub_stiffness_l.publish(zero);
	pub_stiffness_r.publish(zero);
	pub_command_l.publish(q_l_feed);
	pub_command_r.publish(q_r_feed);
	pub_errors.publish(msg_errors);


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

void get_obj_pos( const gazebo_msgs::LinkStates::ConstPtr msg)
{
	Eigen::VectorXd qxo(4);
	// Eigen::VectorXd xo(n_dim_space);
	// Eigen::VectorXd xpo(n_dim_space);

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
	xo[2] = msg->pose[sphere_index].position.z - 1.0; // table height;

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
	// std::cout << "Object Pose   ";
	// for (int i=1; i<6; ++i)
	// 	std::cout << xo(i) << " ";
	// std::cout << std::endl;
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
	return true;
}

