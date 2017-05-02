#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <dual_manipulation_control/SingleRobot.hpp>
// #include <dual_manipulation_control/force_position_control.hpp>
#include <kdl_wrapper/kdl_wrapper.h>
#include <gazebo_msgs/LinkStates.h>


std::vector<float> xo;
Eigen::VectorXf xpo;
int first_time_link_states;
int sphere_index;
std::vector<float> qxo;


void get_obj_pos( const gazebo_msgs::LinkStates::ConstPtr msg)
{
    // Eigen::VectorXf qxo(4);
    // Eigen::VectorXf xo(n_dim_space);
    // Eigen::VectorXf xpo(n_dim_space);

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
    // for (int i=1; i<6; ++i)
    //  std::cout << xo(i) << " ";
    // std::cout << std::endl;
}


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

    Kuka_LWR robotr, robotl;
    KDLWrapper arm_chainr, arm_chainl;

    if (!arm_chainr.init("vito_anchor", "right_arm_7_link"))
    {
        ROS_ERROR("Error initiliazing robot");
    }
    robotr.init_robot("right_arm", arm_chainr.getKDLChain(), 1. / spin_rate);

    first_time_link_states = 1;
    sphere_index = 0;
    if (!arm_chainl.init("vito_anchor", "left_arm_7_link"))
    {
        ROS_ERROR("Error initiliazing robot");
    }
    robotl.init_robot("left_arm", arm_chainl.getKDLChain(), 1. / spin_rate);

    xo.resize(6);
    xo = std::vector<float>(6,0);
    qxo.resize(4);
    qxo = std::vector<float>(4,0);
    qxo[3] = 1.;

    xpo = Eigen::VectorXf::Zero(6);
    // qxo = Eigen::VectorXf::Zero(4);
    // qxo(3) = 1.;

    ros::Publisher pub_tau = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/additional_torque", 0);
    ros::Publisher pub_command = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/command", 0);
    ros::Publisher pub_damping = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/damping", 0);
    ros::Publisher pub_stiffness = node.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/stiffness", 0);
    ros::Publisher pub_activate_controller = node.advertise<std_msgs::Float64>("/activate_controller", 0);
    ros::Publisher pub_error_r = node.advertise<std_msgs::Float64MultiArray>("/error_r", 0);
    ros::Publisher pub_error_l = node.advertise<std_msgs::Float64MultiArray>("/error_l", 0);
    ros::Subscriber sub_joint_states = node.subscribe("/right_arm/joint_states", 100,  &Kuka_LWR::getJointSates, &robotr);


    ros::Publisher pub_taul = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/additional_torque", 0);
    ros::Publisher pub_commandl = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/command", 0);
    ros::Publisher pub_dampingl = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/damping", 0);
    ros::Publisher pub_stiffnessl = node.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/stiffness", 0);
    ros::Subscriber sub_joint_statesl = node.subscribe("/left_arm/joint_states", 100,  &Kuka_LWR::getJointSates, &robotl);

    ros::Subscriber sub_get_obj_pos = node.subscribe("/gazebo/link_states", 100, &get_obj_pos);

    std_msgs::Float64 msg;
    msg.data = 0;

    ros::spinOnce();
    rate.sleep();

    pub_activate_controller.publish(msg);

    ros::spinOnce();
    rate.sleep();

    std_msgs::Float64MultiArray zero, msg_add_torque, msg_add_torquel, msg_error_r, msg_error_l;
    for (int i = 0; i < 7; ++i)
        zero.data.push_back(0.0);

    msg_add_torque.data = zero.data;
    pub_damping.publish(zero);
    pub_stiffness.publish(zero);
    // pub_command.publish(zero);
    pub_tau.publish(zero);

    msg_add_torquel.data = zero.data;
    pub_dampingl.publish(zero);
    pub_stiffnessl.publish(zero);
    // pub_commandl.publish(zero);
    pub_taul.publish(zero);

    ros::spinOnce();
    rate.sleep();
    sleep(1.0);

    robotr.initControl();
    robotl.initControl();


    msg_error_r.data = zero.data;
    msg_error_l.data = zero.data;

    while (first_time_link_states == 1)
    {
        ros::spinOnce();
    }

    KDL::Frame x_des, x;
    x_des = KDL::Frame(KDL::Rotation::Quaternion(qxo[0], qxo[1], qxo[2], qxo[3]), KDL::Vector(xo[0], xo[1] + 0.15, xo[2]));
    robotr.setXReference(x_des);
    std::cout << "right" << std::endl << x_des << std::endl;
    x_des = KDL::Frame(KDL::Rotation::Quaternion(qxo[0], qxo[1], qxo[2], qxo[3]), KDL::Vector(xo[0], xo[1] - 0.15, xo[2]));
    robotl.setXReference(x_des);
    std::cout << "left" << std::endl << x_des << std::endl;
    while (ros::ok())
    {
        robotr.computeControl();
        robotl.computeControl();
        Eigen::VectorXf taur = robotr.getTau();
        Eigen::VectorXf taul = robotl.getTau();
        Eigen::VectorXf errorr = robotr.getErrors();
        Eigen::VectorXf errorl = robotl.getErrors();
        for (int i = 0; i < 7; ++i)
        {
            msg_add_torque.data[i] = taur(i);
            msg_add_torquel.data[i] = taul(i);
            msg_error_r.data[i] = errorr(i);
            msg_error_l.data[i] = errorl(i);
        }
        pub_damping.publish(zero);
        pub_stiffness.publish(zero);
        pub_tau.publish(msg_add_torque);
        pub_taul.publish(msg_add_torquel);
        pub_error_r.publish(msg_error_r);
        pub_error_l.publish(msg_error_l);
        pub_dampingl.publish(zero);
        pub_stiffnessl.publish(zero);
        ros::spinOnce();
        rate.sleep();
    }





    pub_tau.publish(zero);
    ros::spinOnce();

    return 1.0;

}

