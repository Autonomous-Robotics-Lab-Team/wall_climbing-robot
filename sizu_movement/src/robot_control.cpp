#include <robot_control.h>
#include <mutex>


    robot_control::robot_control(): 
    nh_(){  
       
        current_Feet_angle_.setZero();//关节角度
        leg_length_ = {0.0633, 0.164, 0.3227};
        box_length =0.2305;
        box_width =0.2055;
        home_pose_ <<  0, 0, 0, 0,
                       M_PI/4,   -M_PI/4,   M_PI/4,   -M_PI/4,
                       3*M_PI/5, -3*M_PI/5, 3*M_PI/5, -3*M_PI/5;

        leg_xy = leg_length_[0] + leg_length_[1]*cos(home_pose_(1,0)) + leg_length_[2]*sin(M_PI/2 + home_pose_(1,0) - home_pose_(2,0));
 
        last_positions_.setZero();
        initJointStatePublishers();
    }

    void robot_control::get_parameters(double T, double dt) {
        this->T = T;
        this->dt = dt;
    }

    void robot_control::initJointStatePublishers() {

        pub_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>("leg_trajectory_controller/command", 10);
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
        // std::cout<<current_Feet_position<<std::endl;

        for(int i=0;i<4;i++){
            current_Feet_angle_.col(i) = Position_to_Angle(current_Feet_position.col(i));
        }

        current_Feet_angle_.row(0) -= off_leg;
        // std::cout<<current_Feet_angle_<<std::endl;

    }
        
     void robot_control::commandSend(Eigen::Matrix<double,3,4> All_Feet_position, std::vector<int> leg_sign) {
       
        ros::Rate rate(100);
        std::vector<std::string> leg_order = {"FL", "RL", "FR", "RR"};
        std::vector<double> dir_sign= { 1, 1, 1,
                                        1, 1, 1,
                                        1,-1,-1,
                                        1,-1,-1};//实机修正矩阵
        Eigen::Matrix<double,3,4> All_Feet_velocity;
        All_Feet_velocity = (All_Feet_position - last_positions_) / dt;

        inverseKinematics(All_Feet_position);
        // std::cout<<All_Feet_position<<std::endl;

        int j=0;
        for(int i=0;i<4;i++){
            if(leg_sign[j] == i){ 
           point.positions[i*3] = current_Feet_angle_(0,i)*dir_sign[i*3];    
           point.positions[i*3+1] = current_Feet_angle_(1,i)*dir_sign[i*3+1];
           point.positions[i*3+2] = current_Feet_angle_(2,i)*dir_sign[i*3+2];

           // 计算雅可比并用稳健方法解算关节速度（避免直接求逆）
           // 使用当前关节角度（不带 dir_sign 修正）计算雅可比
           Eigen::Matrix<double,3,1> joint_angles = current_Feet_angle_.col(i);
           Eigen::Matrix3d J_local = computeJacobian(joint_angles[0], joint_angles[1], joint_angles[2],
                                                   leg_length_[0], leg_length_[1], leg_length_[2]);

           // 获取当前足端笛卡尔速度（使用实际腿索引，避免列错位导致速度抖动）
           Eigen::Matrix<double,3,1> v_foot = All_Feet_velocity.col(leg_sign[j]);

           Eigen::Matrix<double,3,1> q_dot;
           // 判定雅可比是否接近奇异（用行列式做快速检测）
           const double detJ = J_local.determinant();
           const double det_threshold = 1e-6; // 可调
           if (std::abs(detJ) > det_threshold) {
               // 非奇异：直接用列主元 QR 解算（比直接 inverse 更稳健）
               q_dot = J_local.colPivHouseholderQr().solve(v_foot);
           } else {
               // 近奇异：使用阻尼最小二乘（Damped Least Squares）作为回退
               const double lambda = 1e-2; // 阻尼因子，可根据需要调整
               Eigen::Matrix3d JJt = J_local.transpose() * J_local;
               Eigen::Matrix3d damp = JJt + lambda * lambda * Eigen::Matrix3d::Identity();
               q_dot = damp.ldlt().solve(J_local.transpose() * v_foot);
           }

           // 对关节速度做限制，防止奇异或突变导致过大速度
           const double max_joint_velocity = 2.0; // rad/s，可根据机器人限制调整
           for (int k = 0; k < 3; ++k) {
               if (q_dot[k] > max_joint_velocity) q_dot[k] = max_joint_velocity;
               if (q_dot[k] < -max_joint_velocity) q_dot[k] = -max_joint_velocity;
           }

           // 填充轨迹点速度，同时考虑硬件修正符号 dir_sign
           point.velocities[i*3]   = q_dot[0] * dir_sign[i*3];
           point.velocities[i*3+1] = q_dot[1] * dir_sign[i*3+1];
           point.velocities[i*3+2] = q_dot[2] * dir_sign[i*3+2];

        //    ROS_INFO("Sending command for leg %d: coxa = %.4f, femur = %.4f, tibia = %.4f",
        //     leg_sign[j], point.velocities[i*3], point.velocities[i*3+1], point.velocities[i*3+2]);
           double coxa_deg = point.positions[i*3] * 180 / M_PI;
           double femur_deg = point.positions[i*3+1] * 180 / M_PI;
           double tibia_deg = point.positions[i*3+2] * 180 / M_PI;

           int coxa_scaled = static_cast<int>(coxa_deg * 36 * 100);
           int femur_scaled = static_cast<int>(femur_deg * 36 * 100);
           int tibia_scaled = static_cast<int>(tibia_deg * 36 * 100);

           motor_commands.position[i*3] = coxa_scaled;
           motor_commands.position[i*3+1] = femur_scaled;
           motor_commands.position[i*3+2] = tibia_scaled;

           // 将关节角速度转换为电机命令并做限幅，防止瞬时异常速度引发抖动
           auto clamp_int = [](int32_t v, int32_t limit){
               if (v > limit) return limit;
               if (v < -limit) return -limit;
               return v;
           };
           const int32_t vel_limit = 8000; // 原始刻度限幅，约等于 ~4 rad/s（按下方换算）
           motor_commands.velocity[i*3]   = abs(((int32_t)(point.velocities[i*3]   * 36 * 180 / M_PI), vel_limit));
           motor_commands.velocity[i*3+1] = abs(((int32_t)(point.velocities[i*3+1] * 36 * 180 / M_PI), vel_limit));
           motor_commands.velocity[i*3+2] = abs(((int32_t)(point.velocities[i*3+2] * 36 * 180 / M_PI), vel_limit));

           j++;
           }
        }
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

    void robot_control::stand() {
        Eigen::Matrix<double,3,4> All_Feet_position;
        All_Feet_position.setZero();
        commandSend(All_Feet_position, {0,1,2,3} );
        ros::Duration(0.1).sleep();
    }

    // 计算雅可比矩阵（优化版：缓存三角函数、检查奇异性、使用伪逆）
    Eigen::Matrix3d robot_control::computeJacobian(double theta1, double theta2, double theta3, double l1, double l2, double l3) {
   
        // 预计算常用表达式
        const double pi_4_theta1 = M_PI/4.0 + theta1;
        const double cos_pi4_t1 = cos(pi_4_theta1);
        const double sin_pi4_t1 = sin(pi_4_theta1);
        const double cos_theta2 = cos(theta2);
        const double sin_theta2 = sin(theta2);
        const double theta2_minus_theta3 = theta2 - theta3;
        const double cos_t2mt3 = cos(theta2_minus_theta3);
        const double sin_t2mt3 = sin(theta2_minus_theta3);
        
        // 计算括号内的公共项
        const double bracket_term = l1 + l2 * cos_theta2 + l3 * cos_t2mt3;
        const double sin_sum = l2 * sin_theta2 + l3 * sin_t2mt3;
        
        // 填充雅可比矩阵
        J(0,0) = -sin_pi4_t1 * bracket_term;
        J(0,1) = -cos_pi4_t1 * sin_sum;
        J(0,2) = cos_pi4_t1 * l3 * sin_t2mt3;
        
        J(1,0) = cos_pi4_t1 * bracket_term;
        J(1,1) = -sin_pi4_t1 * sin_sum;
        J(1,2) = sin_pi4_t1 * l3 * sin_t2mt3;
        
        J(2,0) = 0.0;
        J(2,1) = l2 * cos_theta2 + l3 * cos_t2mt3;
        J(2,2) = -l3 * cos_t2mt3;
        
        return J;
}   