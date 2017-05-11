#include <gazebo_msgs/LinkStates.h>

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


void get_states(const sensor_msgs::JointState::ConstPtr msg)
{

    if (first_time_joint_states)
    {
        joint_index = get_joint_index(msg);
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
}

std::vector<int> get_joint_index(const sensor_msgs::JointState::ConstPtr msg)
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

}