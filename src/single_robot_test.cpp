#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <dual_manipulation_control/SingleRobot.hpp>
#include <kdl_wrapper/kdl_wrapper.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gohome");
    ros::NodeHandle node;

    ROS_INFO("[gohome] Node is ready");

    double spin_rate = 500;
    std::string arm;
    ros::param::get("spin_rate", spin_rate);
    node.param<std::string>("arm", arm, "left_arm");
    ROS_DEBUG( "Spin Rate %lf", spin_rate);
    ros::Rate rate(spin_rate);

    Kuka_LWR robot;
    KDLWrapper arm_chain;

    if (!arm_chain.init("vito_anchor", arm + "_7_link"))
    {
        ROS_ERROR("Error initiliazing robot");
    }
    robot.init_robot(arm, arm_chain.getKDLChain(), 1. / spin_rate);

    ros::Publisher pub_tau = node.advertise<std_msgs::Float64MultiArray>("/" + arm  + "/joint_impedance_controller/additional_torque", 0);
    ros::Publisher pub_command = node.advertise<std_msgs::Float64MultiArray>("/" + arm  + "/joint_impedance_controller/command", 0);
    ros::Publisher pub_damping = node.advertise<std_msgs::Float64MultiArray>("/" + arm  + "/joint_impedance_controller/damping", 0);
    ros::Publisher pub_stiffness = node.advertise<std_msgs::Float64MultiArray>("/" + arm  + "/joint_impedance_controller/stiffness", 0);
    ros::Publisher pub_activate_controller = node.advertise<std_msgs::Float64>("/activate_controller", 0);
    ros::Subscriber sub_joint_states = node.subscribe("/" + arm  + "/joint_states", 100,  &Kuka_LWR::getJointSates, &robot);

    std_msgs::Float64 msg;
    msg.data = 0;

    ros::spinOnce();
    rate.sleep();

    pub_activate_controller.publish(msg);

    ros::spinOnce();
    rate.sleep();

    std_msgs::Float64MultiArray zero, msg_add_torque;
    for (int i = 0; i < 7; ++i)
        zero.data.push_back(0.0);

    msg_add_torque.data = zero.data;
    pub_damping.publish(zero);
    pub_stiffness.publish(zero);
    pub_command.publish(zero);
    pub_tau.publish(zero);

    ros::spinOnce();
    rate.sleep();
    sleep(1.0);

    robot.initControl();
    while (ros::ok())
    {
        robot.computeControl();
        Eigen::VectorXf tau = robot.getTau();
        for (int i = 0; i < 7; ++i)
        {
            msg_add_torque.data[i] = tau(i);
        }
        pub_tau.publish(msg_add_torque);
        ros::spinOnce();
        rate.sleep();
    }

    pub_tau.publish(zero);
    ros::spinOnce();

    return 1.0;

}

