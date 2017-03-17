
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
        std::cout << "x_ee_l: \n" << x_ee_l << std::endl;
        std::cout << "x_ee_r: \n" << x_ee_r << std::endl;
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

    sub_joint_states = nh.subscribe("/joint_states", 1, &get_states);
    sub_ft_readings_r = nh.subscribe("/force_torque_r", 1, &get_force_r);
    sub_ft_readings_l = nh.subscribe("/force_torque_l", 1, &get_force_l);
    sub_get_obj_pos = nh.subscribe("/gazebo/link_states", 1, &get_obj_pos);
    ros::Subscriber activate_controller_sub = nh.subscribe("/activate_controller", 1, &activate_controller_cb);

    pub_taul = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/additional_torque", 0);
    pub_taur = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/additional_torque", 0);
    pub_command_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/command", 0);
    pub_command_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/command", 0);
    pub_damping_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/damping", 0);
    pub_damping_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/damping", 0);
    pub_stiffness_l = nh.advertise<std_msgs::Float64MultiArray>("/left_arm/joint_impedance_controller/stiffness", 0);
    pub_stiffness_r = nh.advertise<std_msgs::Float64MultiArray>("/right_arm/joint_impedance_controller/stiffness", 0);

    init_variables(nh);

    sleep(1);
    while (!flag_run_)
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
        ros::spinOnce();
        start = ros::Time::now();
        t_total += dt;

        Eigen::VectorXd G(14);

        G << G_l(0), G_l(1), G_l(2), G_l(3), G_l(4), G_l(5), G_l(6), G_r(0), G_r(1), G_r(2), G_r(3), G_r(4), G_r(5), G_r(6);

        //devo riempire delle matrici e vettori di Eigen per fare le operazioni //

        //dinamica aumentata del manipolatore a due braccia//

        Eigen::VectorXd q(14); //vettore aumentato delle variabili di giunto//

        q << ql[0] , ql[1] , ql[2] , ql[3] , ql[4] , ql[5], ql[6] , qr[0] , qr[1] , qr[2] , qr[3] , qr[4] , qr[5] , qr[6] ;

        Eigen::VectorXd qp(14); //vettore aumentato delle velocità angolari di giunto//

        qp << qpl[0] , qpl[1] , qpl[2] , qpl[3] , qpl[4] , qpl[5], qpl[6] , qpr[0] , qpr[1] , qpr[2] , qpr[3] , qpr[4] , qpr[5] , qpr[6] ;

        Eigen::VectorXd qpp(14); //vettore aumentato delle accelerazioni angolari di giunto//

        qpp << qppl[0] , qppl[1] , qppl[2] , qppl[3] , qppl[4] , qppl[5], qppl[6] , qppr[0] , qppr[1] , qppr[2] , qppr[3] , qppr[4] , qppr[5] , qppr[6] ;

        Eigen::MatrixXd Jleft(6, 7);
        Eigen::MatrixXd Jright(6, 7);

        for (int i = 0 ; i < 6 ; i++) {
            for (int j = 0 ; j < 7 ; j++ ) {
                Jleft(i, j) = J_l(i, j);
                Jright(i, j) = J_r(i, j);
            }
        }

        Eigen::MatrixXd J(12, 14); //jacobiano analitico aumentato//

        J.topLeftCorner(6, 7) = Jleft;
        J.topRightCorner(6, 7) = Eigen::MatrixXd::Zero(6, 7);
        J.bottomLeftCorner(6, 7) = Eigen::MatrixXd::Zero(6, 7);
        J.bottomRightCorner(6, 7) = Jright;

        Eigen::MatrixXd Mleft(7, 7);
        Eigen::MatrixXd Mright(7, 7);

        for (int i = 0 ; i < 7 ; i++) {
            for (int j = 0 ; j < 7 ; j++) {
                Mleft(i, j) = M_l(i, j);
                Mright(i, j) = M_r(i, j);
            }
        }

        Eigen::MatrixXd M(14, 14); //matrice delle inerzie aumentata//

        M.topLeftCorner(7, 7) = Mleft;
        M.topRightCorner(7, 7) = Eigen::MatrixXd::Zero(7, 7);
        M.bottomLeftCorner(7, 7) = Eigen::MatrixXd::Zero(7, 7);
        M.bottomRightCorner(7, 7) = Mright;

        Eigen::VectorXd C(14); //contributi di coriolis + centrifugi augmented//

        C << C_l(0) , C_l(1) , C_l(2) , C_l(3) , C_l(4) , C_l(5) , C_l(6) , C_r(0) , C_r(1) , C_r(2), C_r(3) , C_r(4) , C_r(5) , C_r(6) ;

        //dinamica e cinematica dell'oggetto //

        Eigen::VectorXd r(6);//posizione e orientazione oggetto//

        r << xo(0) , xo(1) , xo(2) , xo(3) , xo(4) , xo(5);

        Eigen::VectorXd rp(6);

        rp = xpo;

        double massa;
        massa = 1;

        Eigen::MatrixXd tensor(3, 3);

        tensor << 0.25 , 0 , 0 ,
               0 , 0.25 , 0 ,
               0 , 0 , 0.25 ;

        Eigen::MatrixXd I3(3, 3);

        I3 = Eigen::MatrixXd::Identity(3, 3);

        Eigen::MatrixXd Ib(6, 6); //matrice delle inerzie per oggetto//

        Ib.topLeftCorner(3, 3) = massa * I3;
        Ib.topRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        Ib.bottomLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        Ib.bottomRightCorner(3, 3) = tensor;

        Eigen::VectorXd Qb(6);
        Eigen::VectorXd w(3);
        Eigen::VectorXd Iw(3);
        Eigen::VectorXd Cross(3);//contributi di Coriolis sull'oggetto// sarebbe w x (Iw)

        w << rp(3) , rp(4) , rp(5);
        Iw << tensor(0, 0)*w(0) - tensor(0, 2)*w(2) , tensor(1, 1)*w(1) , tensor(2, 2)*w(2) - tensor(2, 0)*w(0) ;
        Cross << w(1)*Iw(2) - Iw(1)*w(2) , -w(0)*Iw(2) + Iw(0)*w(2) , w(0)*Iw(1) - Iw(0)*w(1) ;

        Qb << 0 , 0 , 0/*-massa * 9.81*/ , Cross(0) , Cross(1) , Cross(2) ;

        Eigen::MatrixXd Tr(3, 3);

        Tr << 0 , -sin(r[3]) , cos(r[3])*sin(r[4]) ,
        0 , cos(r[3]) , sin(r[3])*sin(r[4]) ,
        1 , 0 , cos(r[4]) ;//jacobiano di trasformazione//

        Eigen::MatrixXd T(6, 6); //trasformazione Velocità-Twist//

        T.topLeftCorner(3, 3) = I3;
        T.topRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        T.bottomLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        T.bottomRightCorner(3, 3) = Tr;

        Eigen::MatrixXd Tp(6, 6);
        Eigen::MatrixXd Tlast(6, 6);

        if (first_step) {
            Tlast = T;
            first_step = 0;
        }

        Tp = (T - Tlast) / dt;

        Eigen::VectorXd Qr(6);

        Qr = Ib * Tp * rp + Qb ;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //mi servono le posizioni(e orientazioni) cartesiane dei 2 end effectors

        Eigen::VectorXd p1(3);
        Eigen::VectorXd p2(3);

        p1 << x_ee_l.p(0) , x_ee_l.p(1) , x_ee_l.p(2);
        p2 << x_ee_r.p(0) , x_ee_r.p(1) , x_ee_r.p(2);

        Eigen::MatrixXd P1(3, 3);
        Eigen::MatrixXd P2(3, 3);

        P1 << 0 , -p1(2) , p1(1) ,
        p1(2) , 0 , -p1(0) ,
        -p1(1) , p1(0) , 0 ;

        P2 << 0 , -p2(2) , p2(1) ,
        p2(2) , 0 , -p2(0) ,
        -p2(1) , p2(0) , 0 ;

        //vincoli cinematici e di forza tra oggetto e manipolatore//

        Eigen::MatrixXd Rc1(3, 3); //matrice di rotazione che mi porta nel primo punto di contatto//

        Rc1 <<  x_ee_l.M.data[0], x_ee_l.M.data[1], x_ee_l.M.data[2],
            x_ee_l.M.data[3], x_ee_l.M.data[4], x_ee_l.M.data[5],
            x_ee_l.M.data[6], x_ee_l.M.data[7], x_ee_l.M.data[8];

        Eigen::MatrixXd Rc2(3, 3); //matrice di rotazione che mi porta nel primo punto di contatto//

        Rc2 <<  x_ee_r.M.data[0], x_ee_r.M.data[1], x_ee_r.M.data[2],
            x_ee_r.M.data[3], x_ee_r.M.data[4], x_ee_r.M.data[5],
            x_ee_r.M.data[6], x_ee_r.M.data[7], x_ee_r.M.data[8];

        Eigen::MatrixXd RC1(6, 6);
        Eigen::MatrixXd RC2(6, 6);

        RC1.topLeftCorner(3, 3) = Rc1;
        RC1.topRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        RC1.bottomLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        RC1.bottomRightCorner(3, 3) = Rc1;

        RC2.topLeftCorner(3, 3) = Rc2;
        RC2.topRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        RC2.bottomLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        RC2.bottomRightCorner(3, 3) = Rc2;

        Eigen::MatrixXd Sc1(6, 3);
        Eigen::MatrixXd Sc2(6, 3);

        Sc1 <<  1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;

        Sc2 <<  1, 0, 0,
            0, 1, 0,
            0, 0, 1,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;

        Eigen::MatrixXd S1(6, 3);
        Eigen::MatrixXd S2(6, 3);

        S1 = RC1 * Sc1;
        S2 = RC2 * Sc2;

        Eigen::MatrixXd S(12, 6);

        S.topLeftCorner(6, 3) = S1;
        S.topRightCorner(6, 3) = Eigen::MatrixXd::Zero(6, 3);
        S.bottomLeftCorner(6, 3) = Eigen::MatrixXd::Zero(6, 3);
        S.bottomRightCorner(6, 3) = S2;

        Eigen::MatrixXd W(6, 12);
        Eigen::MatrixXd W1(6, 6);
        Eigen::MatrixXd W2(6, 6);

        W1.topRightCorner(3, 3) = I3;
        W1.topLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        W1.bottomLeftCorner(3, 3) = P1;
        W1.bottomRightCorner(3, 3) = I3;

        W2.topRightCorner(3, 3) = I3;
        W2.topLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
        W2.bottomLeftCorner(3, 3) = P2;
        W2.bottomRightCorner(3, 3) = I3;

        W.block(0, 0, 6, 6) = W1;
        W.block(0, 5, 6, 6) = W2;

        ///////////////////////// W = W.transpose();//perchè non so se me la giustapposizione avviene in orizzontale o in verticale//

        Eigen::MatrixXd Ws(6, 6);

        Ws = W * S;



        //per noi non ci sono vincoli ambientali quindi nell'articolo Ef = 0 ed E = Ep //

        Eigen::MatrixXd E(6, 6);

        E << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1; //per ora non so com'è fatta ????????//

        Eigen::MatrixXd Elast(6, 6);
        Eigen::MatrixXd Ep(6, 6);

        if (first_step) {
            Elast = E;
            first_step = 0;
        }

        Ep = (E - Elast) / dt;

        Eigen::MatrixXd Einv(6, 6);

        pseudo_inverse(E, Einv);

        //devo scrivere un po di matrici trasposte,derivate e pseudoinverse //

        //////////////////////////////////////////////////////

        Eigen::MatrixXd St(6, 12);

        St = S.transpose();

        Eigen::MatrixXd Stpinv(12, 6);

        pseudo_inverse(St, Stpinv);

        Eigen::MatrixXd S_pinv(6, 12);

        pseudo_inverse(S, S_pinv);

        Eigen::MatrixXd Stp(6, 12);
        Eigen::MatrixXd St_last(6, 12);

        if (first_step) {
            St_last = St;
            first_step = 0;
        }

        Stp = (St - St_last) / dt; //S trasposta e derivata//

        ///////////////////////////////////////////////////////////

        Eigen::MatrixXd Jpinv(14, 12);

        pseudo_inverse(J, Jpinv);

        Eigen::MatrixXd Jp(12, 14);
        Eigen::MatrixXd Jlast(12, 14);

        if (first_step) {
            Jlast = J;
            first_step = 0;
        }

        Jp = (J - Jlast) / dt;

        /////////////////////////////////////////////////////////7

        Eigen::MatrixXd Wst(6, 6);

        Wst = Ws.transpose();

        Eigen::MatrixXd Wstp(6, 6);

        Eigen::MatrixXd Wstlast(6, 6);

        if (first_step) {
            Wstlast = Wst;
            first_step = 0;
        }

        Wstp = (Wst - Wstlast) / dt;

        //////////////////////////////////////////////

        //ora scrivo le due parti della legge di controllo//

        Eigen::VectorXd Tau_A(14);
        Eigen::VectorXd Tau_E(14);
        Eigen::VectorXd Tau_C(14);

        Eigen::VectorXd U1(6);
        Eigen::MatrixXd Kv(6, 6);
        Eigen::MatrixXd Kp(6, 6);
        Eigen::MatrixXd I6(6, 6);

        I6 = Eigen::MatrixXd::Identity(6, 6);

        double kv;
        double kp;

        kv = 0.00001;
        kp = 10;

        Kv = kv * I6;
        Kp = kp * I6;

        Eigen::VectorXd rd(6);
        Eigen::VectorXd rpd(6);
        Eigen::VectorXd rppd(6);

        rd << -0.6, 0, 0.2, 0, 0, 0;
        rpd << 0.0, 0.0, 0.0, 0, 0, 0;
        rppd << 0, 0, 0, 0, 0, 0;

        U1 = rppd + Kv * (rpd - rp) + Kp * (rd - r);
        Eigen::VectorXd rppdes(6);

        rppdes = Einv * (U1 - Ep * rp);

        Eigen::VectorXd qppdes(14);
        Eigen::VectorXd d1(12);
        Eigen::VectorXd d2(14);
        // Eigen::VectorXd d3(6);

        d1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        d2 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; //per ora senza elementi del Kernel dello spazio cinematico e vincolato
        // d3 << 0, 0, 0, 0, 0, 0;

        Eigen::MatrixXd I12(12, 12);
        Eigen::MatrixXd I14(14, 14);

        I12 = Eigen::MatrixXd::Identity(12, 12);
        I14 = Eigen::MatrixXd::Identity(14, 14);

        qppdes = Jpinv * (Stpinv * (Wst * T * rppdes + (Wstp * T + Wst * Tp) * rp - Stp * J * qp) + (I12 - Stpinv * St) * d1 - Jp * qp);// + (I14 - Jpinv * J) * d2 ;

        Eigen::VectorXd ep(12);
        // ep = Eigen::MatrixXd::Zero(12,1);

        Eigen::VectorXd xee_l(6);
        Eigen::VectorXd xee_r(6);

        // std::cout << "Rc1 \n" << Rc1(1);
        xee_l << x_ee_l.p(0) , x_ee_l.p(1) , x_ee_l.p(2) , Rc1(1) , Rc1(2) , Rc1(3);
        xee_r << x_ee_r.p(0) , x_ee_r.p(1) , x_ee_r.p(2) , Rc2(1) , Rc2(2) , Rc2(3);

        // for (int i=0;i<7;++i){
        //         xee_l(i) = x_ee_l.p(i);
        // }

        // for (int i=0;i<7;++i){
        //         xee_r(i) = x_ee_r.p(i);
        // }

        Eigen::VectorXd xpee_l(6);
        Eigen::VectorXd xpee_l_last(6);

        if (first_step) {
            xpee_l_last = xee_l;
            first_step = 0;
            x_ee_l_last = x_ee_l;
        }

        xpee_l = (xee_l - xpee_l_last) / dt;

        Eigen::VectorXd xpee_r(6);
        Eigen::VectorXd xpee_r_last(6);

        if (first_step) {
            xpee_r_last = xee_r;
            first_step = 0;
            x_ee_r_last = x_ee_r;
        }

        xpee_r = (xee_r - xpee_r_last) / dt;

        // init_pose_l.p(1) = init_pose_l.p(1) + .1 * sin(2.0 * M_PI * 1.0 / 10.0 * t_total);
        // init_pose_r.p(1) = init_pose_r.p(1) + .1 * sin(2.0 * M_PI * 1.0 / 10.0 * t_total);
        KDL::Twist pose_error_l = diff(x_ee_l, init_pose_l);
        KDL::Twist pose_error_r = diff(x_ee_r, init_pose_r);

        std::cout << "errorsl: " << pose_error_l(0) << " "
                                << pose_error_l(1) << " "
                                << pose_error_l(2) << " "
                                << pose_error_l(3) << " "
                                << pose_error_l(4) << " "
                                << pose_error_l(5) << std::endl;
        std::cout << "errorsr: " << pose_error_r(0) << " "
                                << pose_error_r(1) << " "
                                << pose_error_r(2) << " "
                                << pose_error_r(3) << " "
                                << pose_error_r(4) << " "
                                << pose_error_r(5) << std::endl;

        KDL::Twist pose_error_derivative_l = diff(x_ee_l, x_ee_l_last) / dt;
        KDL::Twist pose_error_derivative_r = diff(x_ee_r, x_ee_r_last) / dt;

        double kp_control;
        nh.param<double>("/kp", kp_control , 1.0);
        double kv_control;
        nh.param<double>("/kv", kv_control , 0.2);
        Eigen::MatrixXd delta_x = Eigen::MatrixXd::Zero(12,1);
        for (int i = 0; i < 6; ++i)
        {
            // if (i<3){
            ep(i) =  pose_error_l(i);
            ep(i + 6) = pose_error_r(i)*0;
            delta_x(i) =  pose_error_l(i);
            delta_x(i + 6) = pose_error_r(i);
            // }
            // else
            // {
            // ep(i) = 0;
            // ep(i+6) = 0;
            // }
        }
        // ep.block(6,0,6,1) = (rd - r);

        qppdes = Jpinv * ep;
        // qppdes = Eigen::MatrixXd::Zero(12,1);

        // qppdes<< 1,1,1,1,1,1,1,1,1,1,1,1,1,1;
        // Eigen::VectorXd Dq(14);
        Eigen::VectorXd qd(14);
        qd << 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15;
        // for(int i=0; i<14;++i)
        // {
        //     qppdes(i) = 1000.0*(qd(i) - q(i)) ;
        // }

        Eigen::MatrixXd Kx_= Eigen::MatrixXd::Identity(12,12);
        Eigen::MatrixXd Dx_= Eigen::MatrixXd::Identity(12,12);
        // Tau_A = M * qppdes - kp_control*qp + C /*+ G*/ ;
        Tau_A = J.transpose() * (kp_control * Kx_ * delta_x + kp_control * Dx_ * J * qp);

        Eigen::VectorXd Fhsd(6);
        Eigen::MatrixXd Wspinv(6, 6);
        Eigen::MatrixXd Proj(6, 6);
        // Eigen::MatrixXd I6(6,6);

        I6 = Eigen::MatrixXd::Identity(6, 6);

        pseudo_inverse(Ws, Wspinv);

        // Fhsd = Wspinv * (Qr);

        Eigen::MatrixXd Jt(14, 12);
        Jt = J.transpose();

        arma::mat Wspinv_arma(6, 6);
        for (int i = 0 ; i < 6 ; ++i)
        {
            for (int j = 0 ; j < 6 ; ++j)
            {
                Wspinv_arma(i, j) = Wspinv(i, j);
            }

        }

        arma::mat null_Wspinv_arma = arma::null(Wspinv_arma);

        // int nc = null_Wspinv_arma.n_rows;
        Eigen::MatrixXd null_Wspinv(null_Wspinv_arma.n_rows, null_Wspinv_arma.n_cols);

        for (int i = 0 ; i < null_Wspinv_arma.n_rows ; ++i)
        {
            for (int j = 0 ; j < null_Wspinv_arma.n_cols ; ++j)
            {
                null_Wspinv(i, j) = null_Wspinv_arma(i, j);
            }

        }

        Eigen::VectorXd d3(null_Wspinv_arma.n_cols);
        for (int i = 0 ; i < null_Wspinv_arma.n_cols ; ++i)
        {
            d3(i) = 1;
        }

        Fhsd = Wspinv * (Ib * T * rppdes + Qr) + null_Wspinv * d3;

        Tau_E = Jt * S * Fhsd;

        // std::cout << "te\n" << Tau_E << std::endl;


        Tau_C = Tau_A ;//+ Tau_E;

        ////////////////////////////////////////////////

        for (int i = 0 ; i < 7 ; i++) {
            // if (Tau_C(i) < 20) {
            tau_l[i] = Tau_C(i);
            // }
            // else {
            // tau_l[i] = 20;
            // }
        }

        double j;

        for (int i = 7 ; i < 14 ; i++) {
            j = i - 7;
            // if (Tau_C(i) < 20) {

            tau_r[j] = Tau_C(i);
            // }
            // else {
            // tau_r[i] = 20;
            // }
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