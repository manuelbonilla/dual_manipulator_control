
#include "dual_manipulation_control/force_position_control.hpp"


void activate_controller_cb( const std_msgs::Float64::ConstPtr msg)
{
    if (msg->data > 0.0)
    {

        init_pose_r = x_ee_r;
        init_pose_l = x_ee_l;
        init_pose_r.p(0) = xo[0];
        init_pose_r.p(1) = xo[1] + .15;
        init_pose_r.p(2) = xo[2];
        init_pose_l.p(0) = xo[0];
        init_pose_l.p(1) = xo[1] - .15;
        init_pose_l.p(2) = xo[2];
        std::cout << "init_pose_l: \n" << init_pose_l << std::endl;
        std::cout << "init_pose_r: \n" << init_pose_r << std::endl;

        flag_run_ =  true;
    }
    else
    {
        flag_run_ =  false;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_position_control");

    ros::NodeHandle nh("~");

    ros::Subscriber sub_joint_states, sub_ft_readings_r, sub_ft_readings_l, sub_get_obj_pos;

    sub_joint_states = nh.subscribe("/joint_states", 100, &get_states);
    sub_ft_readings_r = nh.subscribe("/force_torque_r", 100, &get_force_r);
    sub_ft_readings_l = nh.subscribe("/force_torque_l", 100, &get_force_l);
    sub_get_obj_pos = nh.subscribe("/gazebo/link_states", 100, &get_obj_pos);
    ros::Subscriber activate_controller_sub = nh.subscribe("/activate_controller", 100, &activate_controller_cb);

    pub_taul = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/additional_torque", 100);
    pub_taur = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/additional_torque", 100);
    pub_command_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/command", 100);
    pub_command_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/command", 100);
    pub_damping_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/damping", 100);
    pub_damping_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/damping", 100);
    pub_stiffness_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/stiffness", 100);
    pub_stiffness_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/stiffness", 100);
    pub_errors = nh.advertise<std_msgs::Float64MultiArray>("/dual_manipulation_control/errors", 100);

    if (!init_variables(nh))
    {
        return false;
    }

    sleep(1);
    while (!flag_run_ && ros::ok())
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
        // ros::spinOnce();
        start = ros::Time::now();
        t_total += dt;

        Eigen::MatrixXd Jleft(n_dim_space, n_joints);
        Eigen::MatrixXd Jright(n_dim_space, n_joints);

        for (int i = 0 ; i < n_dim_space ; i++) {
            for (int j = 0 ; j < n_joints ; j++ ) {
                Jleft(i, j) = J_l(i, j);
                Jright(i, j) = J_r(i, j);
            }
        }

        Eigen::MatrixXd J(n_dim_space * 2, n_joints * 2); //jacobiano analitico aumentato//

        J.topLeftCorner(n_dim_space, n_joints) = Jleft;
        J.topRightCorner(n_dim_space, n_joints) = Eigen::MatrixXd::Zero(n_dim_space, n_joints);
        J.bottomLeftCorner(n_dim_space, n_joints) = Eigen::MatrixXd::Zero(n_dim_space, n_joints);
        J.bottomRightCorner(n_dim_space, n_joints) = Jright;

        Eigen::MatrixXd Mleft(n_joints, n_joints);
        Eigen::MatrixXd Mright(n_joints, n_joints);

        for (int i = 0 ; i < n_joints ; i++) {
            for (int j = 0 ; j < n_joints ; j++) {
                Mleft(i, j) = M_l(i, j);
                Mright(i, j) = M_r(i, j);
            }
        }

        Eigen::MatrixXd M(n_joints*2, n_joints*2); //matrice delle inerzie aumentata//

        M.topLeftCorner(n_joints, n_joints) = Mleft;
        M.topRightCorner(n_joints, n_joints) = Eigen::MatrixXd::Zero(n_joints, n_joints);
        M.bottomLeftCorner(n_joints, n_joints) = Eigen::MatrixXd::Zero(n_joints, n_joints);
        M.bottomRightCorner(n_joints, n_joints) = Mright;

        Eigen::VectorXd qp(n_joints * 2); //vettore aumentato delle velocità angolari di giunto//

        qp << qpl[0] , qpl[1] , qpl[2] , qpl[3] , qpl[4] , qpl[5], qpl[6] , qpr[0] , qpr[1] , qpr[2] , qpr[3] , qpr[4] , qpr[5] , qpr[6] ;


        Eigen::VectorXd Tau_A(n_joints * 2);
        Eigen::VectorXd Tau_E(n_joints * 2);
        Eigen::VectorXd Tau_C(n_joints * 2);


        pose_error_l = diff(x_ee_l, init_pose_l);
        pose_error_r = diff(x_ee_r, init_pose_r);


        double kp_control;
        nh.param<double>("/kp", kp_control , 200.0);
        double kv_control;
        nh.param<double>("/kv", kv_control , 0.2);
        Eigen::MatrixXd delta_x = Eigen::MatrixXd::Zero(n_dim_space * 2, 1);
        for (int i = 0; i < n_dim_space; ++i)
        {
            delta_x(i, 0) =  pose_error_l(i);
            delta_x(i + n_dim_space, 0) = pose_error_r(i);
            if (i >= 3)
            {
                delta_x(i, 0) =  pose_error_l(i) * 0.0;
                delta_x(i + n_dim_space, 0) = pose_error_r(i) * 0.0;
            }


        }

        Eigen::MatrixXd Kx_ = Eigen::MatrixXd::Identity(n_dim_space * 2, n_dim_space * 2);
        Eigen::MatrixXd Dx_ = Eigen::MatrixXd::Identity(n_dim_space * 2, n_dim_space * 2);
        // Tau_A = M * qppdes - kp_control*qp + C /*+ G*/ ;
        Tau_A = J.transpose() * (kp_control * Kx_ * delta_x + kv_control * Dx_ * J * qp);


        Tau_C = Tau_A ;//+ Tau_E;
        ////////////////////////////////////////////////

        for (int i = 0 ; i < n_joints ; i++)
        {
            tau_l[i] = Tau_C(i);
            tau_r[i] = Tau_C(i + n_joints);
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