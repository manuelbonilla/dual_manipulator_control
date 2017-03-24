
#include "dual_manipulation_control/force_position_control.hpp"


void activate_controller_cb( const std_msgs::Float64::ConstPtr msg)
{
    if (msg->data > 0.0)
    {

        init_pose_r = x_ee_r;
        init_pose_l = x_ee_l;
        init_pose_r.p(0) = -0.8;
        init_pose_r.p(1) = 0 + .15;
        init_pose_r.p(2) = 1;
        init_pose_l.p(0) = -0.8;
        init_pose_l.p(1) = 0 - .15;
        init_pose_l.p(2) = 1;
        std::cout << "init_pose_l: \n" << init_pose_l << std::endl;
        std::cout << "init_pose_r: \n" << init_pose_r << std::endl;

        flag_run_ =  true;
    }
    else
    {
        flag_run_ =  false;
    }

    t_total = 0.0;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_position_control");

    ros::NodeHandle nh("~");

    ros::Subscriber sub_joint_states, sub_ft_readings_r, sub_ft_readings_l, sub_get_obj_pos, sub_joint_states_l, sub_joint_states_r;

    sub_joint_states_r = nh.subscribe("/right_arm/joint_states", 100, &get_statesr);
    sub_joint_states_l = nh.subscribe("/left_arm/joint_states", 100, &get_statesl);
    sub_ft_readings_r = nh.subscribe("/force_torque_r", 100, &get_force_r);
    sub_ft_readings_l = nh.subscribe("/force_torque_l", 100, &get_force_l);
    // sub_get_obj_pos = nh.subscribe("/gazebo/link_states", 100, &get_obj_pos);
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
    ros::Rate rate( 1.0 / dt );
    Eigen::VectorXd q_des(n_joints * 2);
    Eigen::VectorXd q(n_joints * 2);
    Eigen::MatrixXd Jleft(n_dim_space, n_joints);
    Eigen::MatrixXd Jright(n_dim_space, n_joints);
    Eigen::MatrixXd J(n_dim_space * 2, n_joints * 2); //jacobiano analitico aumentato//
    Eigen::MatrixXd Jpinv(n_joints * 2, n_dim_space * 2);

    Eigen::MatrixXd Mleft(n_joints, n_joints);
    Eigen::MatrixXd Mright(n_joints, n_joints);
    Eigen::MatrixXd M(n_joints * 2, n_joints * 2); //matrice delle inerzie aumentata//
    Eigen::VectorXd r(6);
    Eigen::VectorXd qp(n_joints * 2); //vettore aumentato delle velocità angolari di giunto//



    Eigen::VectorXd qpp_des(n_joints * 2);



    Eigen::VectorXd Tau_A(n_joints * 2);
    Eigen::VectorXd Tau_E(n_joints * 2);
    Eigen::VectorXd Tau_C(n_joints * 2);

    Eigen::VectorXd err_q(n_joints * 2);
    Eigen::VectorXd err_dq(n_joints * 2);
    Eigen::VectorXd qp_des(n_joints * 2);
    Eigen::VectorXd r_aug(12);
    Eigen::VectorXd rp_aug(12);
    Eigen::VectorXd offset(12);

    Eigen::MatrixXd Jpinv_dot(14, 12);
    Eigen::MatrixXd Jpinv_last(14, 12);

    Eigen::VectorXd C(14);
    int ccc = 0;
    t_total = 0.0;
    q << ql[0] , ql[1] , ql[2] , ql[3] , ql[4] , ql[5], ql[6] , qr[0] , qr[1] , qr[2] , qr[3] , qr[4] , qr[5] , qr[6] ;
    q_des = q;

    double kp_control;
    double kv_control;

    while (ros::ok() && flag_run_)
    {
        // ros::spinOnce();

        start = ros::Time::now();



        for (int i = 0 ; i < n_dim_space ; i++) {
            for (int j = 0 ; j < n_joints ; j++ ) {
                Jleft(i, j) = J_l(i, j);
                Jright(i, j) = J_r(i, j);
            }
        }



        J.topLeftCorner(n_dim_space, n_joints) = Jleft;
        J.topRightCorner(n_dim_space, n_joints) = Eigen::MatrixXd::Zero(n_dim_space, n_joints);
        J.bottomLeftCorner(n_dim_space, n_joints) = Eigen::MatrixXd::Zero(n_dim_space, n_joints);
        J.bottomRightCorner(n_dim_space, n_joints) = Jright;



        pseudo_inverse(J, Jpinv);


        // for (int i = 0 ; i < n_joints ; i++) {
        //     for (int j = 0 ; j < n_joints ; j++) {
        //         Mleft(i, j) = M_l(i, j);
        //         Mright(i, j) = M_r(i, j);
        //     }
        // }



        // C << C_l(0), C_l(1), C_l(2), C_l(3), C_l(4), C_l(5), C_l(6), C_r(0), C_r(1), C_r(2), C_r(3), C_r(4), C_r(5), C_r(6);



        M.topLeftCorner(n_joints, n_joints) = Mleft;
        M.topRightCorner(n_joints, n_joints) = Eigen::MatrixXd::Zero(n_joints, n_joints);
        M.bottomLeftCorner(n_joints, n_joints) = Eigen::MatrixXd::Zero(n_joints, n_joints);
        M.bottomRightCorner(n_joints, n_joints) = Mright;



        r << xo[0], xo[1], xo[2], xo[3], xo[4], xo[5];





        // qpp_des << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        qp << qpl[0] , qpl[1] , qpl[2] , qpl[3] , qpl[4] , qpl[5], qpl[6] , qpr[0] , qpr[1] , qpr[2] , qpr[3] , qpr[4] , qpr[5] , qpr[6] ;

        // if (first_step) {

        //     Jpinv_last = Jpinv;
        //     first_step = 0;

        // }

        // Jpinv_dot = (Jpinv - Jpinv_last) / dt;

        // Eigen::VectorXd F(n_joints * 2);

        // r_aug << xo[0], xo[1], xo[2], xo[3], xo[4], xo[5], xo[0], xo[1], xo[2], xo[3], xo[4], xo[5];
        // // r_aug << 1,0,1,0,0,0,1,0,1,0,0,0;

        // rp_aug << xpo[0], xpo[1], xpo[2], xpo[3], xpo[4], xpo[5], xpo[0], xpo[1], xpo[2], xpo[3], xpo[4], xpo[5];

        // F = Jpinv_dot * r_aug;

        // Eigen::VectorXd Flast(n_joints * 2);

        // if (first_step) {

        //     Flast = F;
        //     first_step = 0;


        // }

        // q_des = Jpinv*r_aug - (F - Flast)*dt;
        // qp_des = Jpinv*rp_aug;

        // err_q = q_des - q;
        // err_dq = qp_des - qp;

        // Eigen::MatrixXd K = Eigen::MatrixXd::Identity(14, 14);

        pose_error_l.vel = (x_ee_l.p - init_pose_l.p) * std::tanh(0.3 * t_total);
        pose_error_l.rot = 0.0 * (init_pose_l.M.UnitX() * x_ee_l.M.UnitX() +
                                  init_pose_l.M.UnitY() * x_ee_l.M.UnitY() +
                                  init_pose_l.M.UnitZ() * x_ee_l.M.UnitZ());

        pose_error_r.vel = (x_ee_r.p - init_pose_r.p) * std::tanh(0.3 * t_total);
        pose_error_r.rot = 0.0 * (init_pose_r.M.UnitX() * x_ee_r.M.UnitX() +
                                  init_pose_r.M.UnitY() * x_ee_r.M.UnitY() +
                                  init_pose_r.M.UnitZ() * x_ee_r.M.UnitZ());



        nh.param<double>("/kp", kp_control , -100.0);

        nh.param<double>("/kv", kv_control , 0.0);
        Eigen::MatrixXd delta_x = Eigen::MatrixXd::Zero(n_dim_space * 2, 1);
        double lambda = 0.5;
        for (int i = 0; i < n_dim_space; ++i)
        {
            delta_x(i, 0) =  pose_error_l(i);
            delta_x(i + n_dim_space, 0) = pose_error_r(i);
        }

        // Eigen::MatrixXd Kx_ = Eigen::MatrixXd::Identity(n_dim_space * 2, n_dim_space * 2);
        // Eigen::MatrixXd Dx_ = Eigen::MatrixXd::Identity(n_dim_space * 2, n_dim_space * 2);


        // for (int i = 0; i < 3; ++i)
        // {
        //     Kx_(3 + i, 3 + i) = 0.01;
        //     Kx_(9 + i, 9 + i) = 0.01;
        // }

        // Eigen::MatrixXd Er =  Eigen::MatrixXd::Zero(12, 1);
        // Er = (kp_control * Kx_ * delta_x + kv_control * Dx_ * J * qp);

        qp_des = Eigen::VectorXd::Zero(14);
        qp_des = 0.1 * Jpinv * delta_x;
        q_des += qp_des * dt;
        // Tau_A = J.transpose() * Er;
        // q_des << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        // qp_des = Eigen::VectorXd::Zero(14);
        Tau_A = -kp_control * (q_des - q) - kv_control * (qp_des - qp);

        // std::cout << (q_des - q).transpose() << std::endl;
        // std::cout << Tau_A.transpose() << std::endl;


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
        t_total += dt;

        rate.sleep();
        ros::spinOnce();
        finish = ros::Time::now();
        // dt = finish.toSec() - start.toSec();

        ccc++;
    } // end while

    return 0;
} // end main