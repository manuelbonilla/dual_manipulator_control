#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <vector>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gohome");
	ros::NodeHandle node;

	ROS_INFO("[gohome] Node is ready");

	double spin_rate = 100;
	ros::param::get("~spin_rate", spin_rate);
	ROS_DEBUG( "Spin Rate %lf", spin_rate);
	ros::Rate rate(spin_rate);

	ros::Publisher pub_taul = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/additional_torque", 0);
    ros::Publisher pub_taur = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/additional_torque", 0);
    ros::Publisher pub_command_l = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/command", 0);
    ros::Publisher pub_command_r = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/command", 0);
    ros::Publisher pub_damping_l = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/damping", 0);
    ros::Publisher pub_damping_r = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/damping", 0);
    ros::Publisher pub_stiffness_l = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/stiffness", 0);
    ros::Publisher pub_stiffness_r = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/stiffness", 0);
    ros::Publisher pub_activate_controller = node.advertise<std_msgs::Float64>("/activate_controller", 0);

    std_msgs::Float64 msg;
    msg.data = 0;

	ros::spinOnce();
    rate.sleep();

    pub_activate_controller.publish(msg);

    ros::spinOnce();
    rate.sleep();

    usleep(3*1000000);

	std::vector<double> q_i_l = {0.77, 0.46, 0.0, -1.43, 0.0, 0.92,  0.0};
	std::vector<double> q_i_r = {-0.77, 0.83, 0.0, -1.42, 0.0, 0.92, 0.0};

    std_msgs::Float64MultiArray zero, q_l_feed, q_r_feed, stiffness, damping, add_torque;
    for (int i=0; i<7; ++i)
    {
        stiffness.data.push_back(300.0);
        damping.data.push_back(0.5);
        add_torque.data.push_back(0.0);
        q_l_feed.data.push_back(q_i_l[i]);
        q_r_feed.data.push_back(q_i_r[i]);
    }

    pub_damping_l.publish(damping);
    pub_damping_r.publish(damping);
    pub_stiffness_l.publish(stiffness);
    pub_stiffness_r.publish(stiffness);
    pub_command_l.publish(q_l_feed);
    pub_command_r.publish(q_r_feed);
    pub_taul.publish(add_torque);
    pub_taur.publish(add_torque);

	ros::spinOnce();
    rate.sleep();

    return 1.0;

}

