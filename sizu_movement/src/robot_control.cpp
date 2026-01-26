#include <robot_control.h>
#include <mutex>


    robot_control::robot_control(): 
    nh_(){  
       
        current_Feet_angle_.setZero();//关节角度
        leg_length_ = {0.0767, 0.1631, 0.3023};
        box_length =0.2305;
        box_width =0.2055;
        home_pose_ <<  0, 0, 0, 0,
                       M_PI/4,   -M_PI/4,   M_PI/4,   -M_PI/4,
                       2*M_PI/3, -2*M_PI/3, 2*M_PI/3, -2*M_PI/3;

        leg_xy = leg_length_[0] + leg_length_[1]*cos(home_pose_(1,0)) + leg_length_[2]*sin(M_PI/2 + home_pose_(1,0) - home_pose_(2,0));
 
        last_positions_.setZero();
        initJointStatePublishers();
    }

    void robot_control::get_parameters(double T, double dt) {
        this->T = T;
        this->dt = dt;
    }

    void robot_control::initJointStatePublishers() {

        pub_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/sizu/leg_trajectory_controller/command", 10);
        // traj_msg.header.stamp = ros::Time::now();
        traj_msg.joint_names = {
            "coxa_joint_FL", "femur_joint_FL", "tibia_joint_FL",
            "coxa_joint_RL", "femur_joint_RL", "tibia_joint_RL",
            "coxa_joint_FR", "femur_joint_FR", "tibia_joint_FR",
            "coxa_joint_RR", "femur_joint_RR", "tibia_joint_RR"
        };
        traj_msg.points.clear();
        point.positions.resize(12, 0.0);
        point.velocities.resize(12, 0.0);
        // point.time_from_start = ros::Duration(dt);

        pub_motor_commands_ = nh_.advertise<sensor_msgs::JointState>("motor_commands", 10);
        motor_commands.header = std_msgs::Header();
        // motor_commands.header.stamp = ros::Time::now();
        motor_commands.name = {
            "coxa_FL", "femur_FL", "tibia_FL",
            "coxa_RL", "femur_RL", "tibia_RL",
            "coxa_FR", "femur_FR", "tibia_FR",
            "coxa_RR", "femur_RR", "tibia_RR"
        };
        motor_commands.position.resize(12, 0);
        motor_commands.velocity.resize(12, 0);
        
        // 验证nh_是否初始化成功
        if(!nh_.ok()) {
            ROS_ERROR("NodeHandle initialization failed!");
        }
    }


    // 计算腿部关节角度
    Eigen::Matrix<double,3,1> robot_control::Position_to_Angle(Eigen::Matrix<double,3,1> position) {
      Eigen::Matrix<double,3,1> angle;
      double l=sqrt(position[0]*position[0]+position[1]*position[1])-leg_length_[0];//投影长度
      double l_=sqrt(l*l+position[2]*position[2]);
      angle(0) = atan2(position[1], position[0]);
      angle(1) = acos((l_*l_+leg_length_[1]*leg_length_[1]-leg_length_[2]*leg_length_[2])/(2*l_*leg_length_[1]))+atan2(position[2], l);
      angle(2) = M_PI-acos((leg_length_[1]*leg_length_[1]+leg_length_[2]*leg_length_[2]-l_*l_)/(2*leg_length_[1]*leg_length_[2]));

      return angle;
    }
    
    //坐标转换，位置转角度
    void robot_control::inverseKinematics(Eigen::Matrix<double,3,4> Feet_position) {

        Eigen::Matrix<double,1,4> off_leg;
        off_leg << M_PI/4, 3*M_PI/4, -M_PI/4, -3*M_PI/4; //腿部外展角偏移量
        double h=leg_length_[2]*cos(M_PI/2 + home_pose_(1,0) - home_pose_(2,0)) - leg_length_[1]*sin(home_pose_(1,0)); //初始抬高高度
        init_Feet_position << leg_xy*cos(M_PI/4), -leg_xy*cos(M_PI/4),  leg_xy*cos(M_PI/4), -leg_xy*cos(M_PI/4),
                              leg_xy*sin(M_PI/4),  leg_xy*sin(M_PI/4), -leg_xy*sin(M_PI/4), -leg_xy*sin(M_PI/4),
                              -h, -h, -h, -h;//初始足端位置0
        Eigen::Matrix<double,3,4> current_Feet_position = Feet_position + init_Feet_position;
        // std::cout<<init_Feet_position<<std::endl;

        for(int i=0;i<4;i++){
            current_Feet_angle_.col(i) = Position_to_Angle(current_Feet_position.col(i));
        }

        current_Feet_angle_.row(0) -= off_leg;
        // std::cout<<current_Feet_angle_<<std::endl;

    }
        
    // 计算雅可比矩阵（优化版）
Eigen::Matrix3d robot_control::computeJacobian(double theta1, double theta2, double theta3, 
                                               double l1, double l2, double l3) {
    // 预计算常用三角函数值
    const double theta1_offset = M_PI/4.0 + theta1;
    const double cos_theta1_offset = cos(theta1_offset);
    const double sin_theta1_offset = sin(theta1_offset);
    
    const double cos_theta2 = cos(theta2);
    const double sin_theta2 = sin(theta2);
    
    const double delta_theta = theta2 - theta3;
    const double cos_delta = cos(delta_theta);
    const double sin_delta = sin(delta_theta);
    
    // 计算公共项
    const double R = l1 + l2 * cos_theta2 + l3 * cos_delta;
    const double sin_sum = l2 * sin_theta2 + l3 * sin_delta;
    const double cos_sum = l2 * cos_theta2 + l3 * cos_delta;
    
    // 填充雅可比矩阵
    Eigen::Matrix3d J;
    
    // 第一行: ∂x/∂θi
    J(0, 0) = -sin_theta1_offset * R;
    J(0, 1) = -cos_theta1_offset * sin_sum;
    J(0, 2) = cos_theta1_offset * l3 * sin_delta;
    
    // 第二行: ∂y/∂θi
    J(1, 0) = cos_theta1_offset * R;
    J(1, 1) = -sin_theta1_offset * sin_sum;
    J(1, 2) = sin_theta1_offset * l3 * sin_delta;
    
    // 第三行: ∂z/∂θi
    J(2, 0) = 0.0;
    J(2, 1) = cos_sum;
    J(2, 2) = -l3 * cos_delta;
    
    return J;
}

// 计算关节速度（带奇异性处理）
Eigen::Matrix<double,3,1> robot_control::calculateJointVelocities(const Eigen::Matrix<double,3,1>& joint_angles, const Eigen::Matrix<double,3,1>& cartesian_velocity) {
    
    // 获取腿长参数
    double l1 = leg_length_[0];
    double l2 = leg_length_[1];
    double l3 = leg_length_[2];
    
    // 计算雅可比矩阵
    Eigen::Matrix3d J = computeJacobian(
        joint_angles[0], joint_angles[1], joint_angles[2], 
        l1, l2, l3
    );
    
    // 使用SVD进行稳健求解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // 获取奇异值
    Eigen::Vector3d singular_values = svd.singularValues();
    
    // 设置奇异值阈值
    const double epsilon = 1e-6;
    
    // 计算阻尼因子（自适应阻尼）
    double lambda_sq = 0.0;
    double min_singular_value = singular_values.minCoeff();
    
    if (min_singular_value < epsilon) {
        lambda_sq = (1.0 - min_singular_value / epsilon) * 0.01;
    }
    
    // 使用阻尼最小二乘求解
    Eigen::Matrix3d JJT = J * J.transpose();
    Eigen::Matrix3d damped = JJT + lambda_sq * Eigen::Matrix3d::Identity();
    Eigen::Vector3d joint_velocities = J.transpose() * damped.ldlt().solve(cartesian_velocity);
    
    // 关节速度限幅
    const double max_velocity = 2.0; // rad/s
    for (int i = 0; i < 3; ++i) {
        if (joint_velocities[i] > max_velocity) joint_velocities[i] = max_velocity;
        if (joint_velocities[i] < -max_velocity) joint_velocities[i] = -max_velocity;
    }
    
    return joint_velocities;
}

// 发送命令到所有腿部
void robot_control::commandSend(Eigen::Matrix<double,3,4> All_Feet_position, std::vector<int> leg_sign) {
    ros::Rate rate(100);
    
    // 方向修正矩阵
    std::vector<double> dir_sign = { 
        1, -1, 1,    // FL
        1, 1, -1,    // RL
        1, 1, -1,    // FR
        1, -1, 1     // RR
    };

    std::vector<double> dir = { 1, 1, -1,
                                1, 1, -1,
                                1, 1, -1,
                                1, 1, -1 };
    
    // 计算足端速度
    Eigen::Matrix<double,3,4> All_Feet_velocity = (All_Feet_position - last_positions_) / dt;
    
    // 计算逆运动学
    inverseKinematics(All_Feet_position);
    
    // 处理每条腿
    int j = 0;
    for(int i = 0; i < 4; i++) {
        if(leg_sign[j] == i) { 
            // 设置关节位置
            point.positions[i*3]     = current_Feet_angle_(0, i) * dir_sign[i*3];
            point.positions[i*3+1]   = current_Feet_angle_(1, i) * dir_sign[i*3+1];
            point.positions[i*3+2]   = current_Feet_angle_(2, i) * dir_sign[i*3+2];
            
            // 获取当前关节角度（无符号修正）
            Eigen::Matrix<double,3,1> joint_angles = current_Feet_angle_.col(i);
            
            // 获取当前足端笛卡尔速度
            Eigen::Matrix<double,3,1> v_foot = All_Feet_velocity.col(leg_sign[j]);
            
            // 计算关节速度
            Eigen::Matrix<double,3,1> q_dot = calculateJointVelocities(joint_angles, v_foot);
            
            // 设置关节速度（考虑硬件修正符号）
            point.velocities[i*3]   = q_dot[0] * dir_sign[i*3];
            point.velocities[i*3+1] = q_dot[1] * dir_sign[i*3+1];
            point.velocities[i*3+2] = q_dot[2] * dir_sign[i*3+2];
            
            // 转换为角度并缩放
            double coxa_deg = point.positions[i*3] * 180 / M_PI;
            double femur_deg = point.positions[i*3+1] * 180 / M_PI;
            double tibia_deg = point.positions[i*3+2] * 180 / M_PI;
            
            // 电机命令转换（刻度转换）
            int coxa_scaled = static_cast<int>(coxa_deg * 36 * 100);
            int femur_scaled = static_cast<int>(femur_deg * 36 * 100);
            int tibia_scaled = static_cast<int>(tibia_deg * 36 * 100);
            
            motor_commands.position[i*3] = coxa_scaled*dir[i*3];
            motor_commands.position[i*3+1] = femur_scaled*dir[i*3+1];
            motor_commands.position[i*3+2] = tibia_scaled*dir[i*3+2];
            
            // 速度限幅函数
            auto clamp_int = [](int32_t v, int32_t max_limit, int32_t min_limit = 0) {
                if (v > max_limit) return max_limit;
                if (v < -max_limit) return -max_limit;
                if (v > -min_limit && v < min_limit) return min_limit;
                return v;
            };
            
            // 速度转换和限幅
            const int32_t vel_limit = 4000;
            
            int32_t coxa_vel = static_cast<int32_t>(point.velocities[i*3] * 36 * 180 / M_PI);
            int32_t femur_vel = static_cast<int32_t>(point.velocities[i*3+1] * 36 * 180 / M_PI);
            int32_t tibia_vel = static_cast<int32_t>(point.velocities[i*3+2] * 36 * 180 / M_PI);
            
            motor_commands.velocity[i*3]   = abs(clamp_int(coxa_vel, vel_limit));
            motor_commands.velocity[i*3+1] = abs(clamp_int(femur_vel, vel_limit));
            motor_commands.velocity[i*3+2] = abs(clamp_int(tibia_vel, vel_limit));
            // ROS_INFO_STREAM("Leg " << i << " Commanded Positions (scaled): "
            //                  << motor_commands.position[i*3] << ", "
            //                  << motor_commands.position[i*3+1] << ", "
            //                  << motor_commands.position[i*3+2]);
            ROS_INFO_STREAM("Leg " << i << " Commanded Velocities (scaled): "
                             << motor_commands.velocity[i*3] << ", "
                             << motor_commands.velocity[i*3+1] << ", "
                             << motor_commands.velocity[i*3+2]);
            j++;
        }
    }
    
    // 保存当前位置
    last_positions_ = All_Feet_position;
    
    // 发布电机命令
    pub_motor_commands_.publish(motor_commands);
    
    // 发布轨迹命令
    traj_msg.points.clear();
    point.time_from_start = ros::Duration(dt);
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.points.push_back(point);
    pub_trajectory_.publish(traj_msg);
    
    rate.sleep();
}

// 站立函数
void robot_control::stand() {
    Eigen::Matrix<double,3,4> All_Feet_position;
    All_Feet_position.setZero();
    commandSend(All_Feet_position, {0,1,2,3} );
    ros::Duration(0.1).sleep();
}