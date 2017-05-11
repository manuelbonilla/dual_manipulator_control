#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <dual_manipulation_control/SingleRobot.hpp>
// #include <dual_manipulation_control/ros_server.hpp>
// #include <dual_manipulation_control/force_position_control.hpp>
#include <kdl_wrapper/kdl_wrapper.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/WrenchStamped.h>

std::vector<float> xo;
Eigen::VectorXf xpo;
int first_time_link_states;
int sphere_index;
std::vector<float> qxo;

void get_obj_pos( const gazebo_msgs::LinkStates::ConstPtr msg)
{
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

    if ((2 * (qxo[0]*qxo[1] + qxo[2]*qxo[3])) >= 0) {
        xo[3] = atan(2 * (qxo[0] * qxo[1] + qxo[2] * qxo[3]) / (1 - 2 * (qxo[1] * qxo[1] + qxo[2] * qxo[2])));
    } else {
        xo[3] = atan(2 * (qxo[0] * qxo[1] + qxo[2] * qxo[3]) / (1 - 2 * (qxo[1] * qxo[1] + qxo[2] * qxo[2]))) + 3.14;
    }

    xo[4] = asin(2 * (qxo[0] * qxo[2] - qxo[3] * qxo[1]));

    if ((2 * (qxo[0]*qxo[3] + qxo[1]*qxo[2])) >= 0) {
        xo[5] = atan(2 * (qxo[0] * qxo[3] + qxo[1] * qxo[2]) / (1 - 2 * (qxo[2] * qxo[2] + qxo[3] * qxo[3])));
    } else {
        xo[5] = atan(2 * (qxo[0] * qxo[3] + qxo[1] * qxo[2]) / (1 - 2 * (qxo[2] * qxo[2] + qxo[3] * qxo[3]))) + 3.14;
    }

    xpo(0) = msg->twist[sphere_index].linear.x;
    xpo(1) = msg->twist[sphere_index].linear.y;
    xpo(2) = msg->twist[sphere_index].linear.z;;
    xpo(3) = msg->twist[sphere_index].angular.x;
    xpo(4) = msg->twist[sphere_index].angular.y;
    xpo(5) = msg->twist[sphere_index].angular.z;
    // std::cout << "Object Pose   ";
    // std::cout << xo(i) << " ";
    // std::cout << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ftsensor");
    ros::NodeHandle node;

    ROS_INFO("[gohome] Node is ready");

    double spin_rate = 500;
    std::string arm;
    ros::param::get("spin_rate", spin_rate);
    node.param<std::string>("arm", arm, "left_arm");
    ROS_DEBUG( "Spin Rate %lf", spin_rate);
    ros::Rate rate(spin_rate);

    Kuka_LWR robotr, robotl;
    KDLWrapper arm_chainr, arm_chainl;

    if (!arm_chainr.init("vito_anchor", "right_arm_7_link"))
    {
        ROS_ERROR("Error initializing robot");
    }
    robotr.init_robot("right_arm", arm_chainr.getKDLChain(), 1. / spin_rate);

    first_time_link_states = 1;
    sphere_index = 0;
    if (!arm_chainl.init("vito_anchor", "left_arm_7_link"))
    {
        ROS_ERROR("Error initializing robot");
    }
    robotl.init_robot("left_arm", arm_chainl.getKDLChain(), 1. / spin_rate);

    ros::Subscriber sub_joint_states = node.subscribe("/right_arm/joint_states", 100,  &Kuka_LWR::getJointSates, &robotr);
    ros::Subscriber sub_joint_statesl = node.subscribe("/left_arm/joint_states", 100,  &Kuka_LWR::getJointSates, &robotl);
    ros::Subscriber sub_get_obj_pos = node.subscribe("/gazebo/link_states", 100, &get_obj_pos);

    ros::Publisher pub_force_r = node.advertise<geometry_msgs::WrenchStamped>("/force_right", 100);
    ros::Publisher pub_force_l = node.advertise<geometry_msgs::WrenchStamped>("/force_left", 100);

    xo.resize(6);
    xo = std::vector<float>(6, 0);
    KDL::Frame x_des, x;
    qxo.resize(4);
    qxo = std::vector<float>(4, 0);
    qxo[3] = 1.;

    xpo = Eigen::VectorXf::Zero(6);

    ros::spinOnce();
    rate.sleep();

    x_des = KDL::Frame(KDL::Rotation::Quaternion(std::sqrt(2.) / 2., qxo[1], qxo[2], std::sqrt(2.) / 2.), KDL::Vector(xo[0], xo[1] + 0.15 / 2., xo[2]));
    robotr.setXReference(x_des);
    // std::cout << "right" << std::endl << x_des << std::endl;
    x_des = KDL::Frame(KDL::Rotation::Quaternion(-std::sqrt(2.) / 2., qxo[1], qxo[2], std::sqrt(2.) / 2.), KDL::Vector(xo[0], xo[1] - 0.15 / 2., xo[2]));
    robotl.setXReference(x_des);
    // std::cout << "left" << std::endl << x_des << std::endl;
    // std::cout << "Approaching 2: ";

    Eigen::MatrixXf K;
    K = Eigen::MatrixXf::Zero(6, 6); // contact stiffness

    float stiffness(1000), scale_t(.1);
    K(0, 0) = stiffness; K(1, 1) = stiffness; K(2, 2) = stiffness;
    K(3, 3) = stiffness * scale_t; K(4, 4) = stiffness * scale_t; K(5, 5) = stiffness * scale_t;
    while (ros::ok())
    {
        x_des = KDL::Frame(KDL::Rotation::Quaternion(std::sqrt(2.) / 2., qxo[1], qxo[2], std::sqrt(2.) / 2.), KDL::Vector(xo[0], xo[1] + 0.15 / 2. + 0.02, xo[2]));
        robotr.setXReference(x_des);
        // std::cout << "right" << std::endl << x_des << std::endl;
        x_des = KDL::Frame(KDL::Rotation::Quaternion(-std::sqrt(2.) / 2., qxo[1], qxo[2], std::sqrt(2.) / 2.), KDL::Vector(xo[0], xo[1] - 0.15 / 2. - 0.02,  xo[2]));
        robotl.setXReference(x_des);
        // std::cout << "left" << std::endl << x_des << std::endl;
        // std::cout << "Approaching 2: ";
        robotr.computeControl();
        robotl.computeControl();

        Eigen::MatrixXf EEpose_r =  robotr.getEEPose();
        Eigen::MatrixXf EEpose_l =  robotl.getEEPose();

        Eigen::MatrixXf T_7_c_r = EEpose_r.inverse() * robotr.getXReference();
        Eigen::MatrixXf T_7_c_l = EEpose_l.inverse() * robotl.getXReference();

        Eigen::VectorXf err_r = robotr.getCartErr();
        Eigen::VectorXf err_l = robotl.getCartErr();

        err_r(0) = T_7_c_r(0, 3); err_r(1) = T_7_c_r(1, 3); err_r(2) = T_7_c_r(2, 3);
        err_l(0) = T_7_c_l(0, 3); err_l(1) = T_7_c_l(1, 3); err_l(2) = T_7_c_l(2, 3);

        Eigen::MatrixXf d(4, 4);
        d = Eigen::MatrixXf::Identity(4, 4);
        Eigen::MatrixXf a(1,1);
        a =  (d.block<3,1>(0, 0).transpose() * T_7_c_r.block<3,1>(0, 0));
        err_r(3) = 0.5 * a(0, 0);
        a =  (d.block<3,1>(0, 1).transpose() * T_7_c_r.block<3,1>(0, 1));
        err_r(4) = 0.5 * a(0, 0);
        a =  (d.block<3,1>(0, 2).transpose() * T_7_c_r.block<3,1>(0, 2));
        err_r(5) = 0.5 * a(0, 0);

        a =  (d.block<3,1>(0, 0).transpose() * T_7_c_l.block<3,1>(0, 0));
        err_l(3) = 0.5 * a(0, 0);
        a =  (d.block<3,1>(0, 1).transpose() * T_7_c_l.block<3,1>(0, 1));
        err_l(4) = 0.5 * a(0, 0);
        a =  (d.block<3,1>(0, 2).transpose() * T_7_c_l.block<3,1>(0, 2));
        err_l(5) = 0.5 * a(0, 0);

        float n_r = std::sqrt(err_r(0) * err_r(0) + err_r(1) * err_r(1) + err_r(2) * err_r(2) );
        float n_l = std::sqrt(err_l(0) * err_l(0) + err_l(1) * err_l(1) + err_l(2) * err_l(2) );

        geometry_msgs::WrenchStamped msg_f_r;
        geometry_msgs::WrenchStamped msg_f_l;
        Eigen::VectorXf F_r = Eigen::VectorXf::Zero(6);
        Eigen::VectorXf F_l = Eigen::VectorXf::Zero(6);
        // std::cout << "error r: " << err_r.transpose() << std::endl;
        // std::cout << "error l: " << err_l.transpose() << std::endl;
        // std::cout << "n_r: " << n_r << " n_l: " << n_l << std::endl;
        // std::cout << "Object pose: " << xo[0] << ", " << xo[1] << ", " << xo[2] << std::endl;
        float force_threshold(0.05);
       if (n_r <= force_threshold)
        {
            //compute force
            F_r = -K * err_r;
            //publish force
        }
        if (n_l <= force_threshold)
        {
            //compute force
            F_l = -K * err_l;
            //publish force
        }

        msg_f_r.wrench.force.x = F_r(0);
        msg_f_r.wrench.force.y = F_r(1);
        msg_f_r.wrench.force.z = F_r(2);
        msg_f_r.wrench.torque.x = F_r(3);
        msg_f_r.wrench.torque.y = F_r(4);
        msg_f_r.wrench.torque.z = F_r(5);

        msg_f_r.header.stamp = ros::Time::now();
        msg_f_r.header.frame_id = "right_arm_7_link";

        pub_force_r.publish(msg_f_r);

        msg_f_l.wrench.force.x = F_l(0);
        msg_f_l.wrench.force.y = F_l(1);
        msg_f_l.wrench.force.z = F_l(2);
        msg_f_l.wrench.torque.x = F_l(3);
        msg_f_l.wrench.torque.y = F_l(4);
        msg_f_l.wrench.torque.z = F_l(5);
        msg_f_l.header.stamp = ros::Time::now();
        msg_f_l.header.frame_id = "left_arm_7_link";
        pub_force_l.publish(msg_f_l);

        ros::spinOnce();
        rate.sleep();
    }

    // robotl.enableForceControl(1.);
    // robotr.enableForceControl(1.);


    // pub_tau.publish(zero);
    ros::spinOnce();

    return 1.0;

}

