#include "EstimateParameters.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

    EstimateParameters::EstimateParameters() : nh() {
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
        keyboard_active_ = false;
        key_timeout_ = 0.5;
        last_key_ = '\0';

        body_position_world.setZero();
        body_velocity_world.setZero();
        body_acceleration_world.setZero();
        body_angular.setZero();
        body_angular_velocity.setZero();
        All_Feet_position.setZero();

        FL_position_world.setZero();
        FL_position_body.setZero();
        FL_velocity_world.setZero();
        FL_velocity_body.setZero();
        FL_angle_body.setZero();

        RL_position_world.setZero();
        RL_position_body.setZero();
        RL_velocity_world.setZero();
        RL_velocity_body.setZero();
        RL_angle_body.setZero();

        FR_position_world.setZero();
        FR_position_body.setZero();
        FR_velocity_world.setZero();
        FR_velocity_body.setZero();
        FR_angle_body.setZero();

        RR_position_world.setZero();
        RR_position_body.setZero();
        RR_velocity_world.setZero();
        RR_velocity_body.setZero();
        RR_angle_body.setZero();

        eulerAngle.setZero();
        quaternion.setIdentity();
        R_world_to_body.setIdentity();
        loop_dt_ = 0.0;
        T = 1.2;
        dt = 0.3;
        compensation_velocity = 1.164;
        high_hop = 0.06;
        _currentTime = ros::Time(0);
        _lastTime = ros::Time(0);
        last_key_time_ = ros::Time::now();
        last_cmd_time_ = ros::Time::now();

        init_position <<  0.19111, -0.19111,  0.19111, -0.19111,
                          0.19111,  0.19111, -0.19111, -0.19111,
                         -0.17667, -0.17667, -0.17667, -0.17667;

        nh.param<std::string>("imu_topic", imu_topic_, std::string("/imu"));
        nh.param<bool>("use_imu_orientation", use_imu_orientation_, true);
        nh.param<std::string>("odom_frame_id", odom_frame_id_, std::string("odom"));
        nh.param<std::string>("base_frame_id", base_frame_id_, std::string("base_link"));
        nh.param("yaw_gain", yaw_gain_, 1.0);
        nh.param("lin_gain", lin_gain_, 1.0);

        cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &EstimateParameters::CallBack, this);
        imu_sub = nh.subscribe(imu_topic_, 50, &EstimateParameters::ImuCb, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
        keyboard_thread_= std::thread(&EstimateParameters::keyboardListenerThread, this);
    }
    
    EstimateParameters::~EstimateParameters() {
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
        nh.shutdown();
    }

    void EstimateParameters::CallBack(const geometry_msgs::Twist::ConstPtr& msg) {
        Eigen::Vector3d lin, ang;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (keyboard_active_) {
                return;
            }
            Eigen::Vector3d raw_lin(msg->linear.x, msg->linear.y, msg->linear.z);
            Eigen::Vector3d raw_ang(msg->angular.x, msg->angular.y, msg->angular.z);

            ROS_INFO_STREAM("lin: " << raw_lin.transpose() << ", ang: " << raw_ang.transpose());

            smoothed_lin_cmd_ = smooth_alpha_*raw_lin + (1.0 - smooth_alpha_)*smoothed_lin_cmd_;
            smoothed_ang_cmd_ = smooth_alpha_*raw_ang + (1.0 - smooth_alpha_)*smoothed_ang_cmd_;


            body_velocity_world = lin_gain_ * smoothed_lin_cmd_;
            body_angular_velocity = smoothed_ang_cmd_;
            lin = body_velocity_world;
            ang = body_angular_velocity;
            last_cmd_time_ = ros::Time::now();
        }
        const double lin_diff = (lin - prev_lin_vel_).norm();
        const double ang_diff = (ang - prev_ang_vel_).norm();
        if (first_log_ || lin_diff > 1e-4 || ang_diff > 1e-4) {
            first_log_ = false;
            prev_lin_vel_ = lin;
            prev_ang_vel_ = ang;
            
        }
    }

    void EstimateParameters::ImuCb(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        imu_quat_.w() = msg->orientation.w;
        imu_quat_.x() = msg->orientation.x;
        imu_quat_.y() = msg->orientation.y;
        imu_quat_.z() = msg->orientation.z;
        imu_ang_vel_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        imu_received_ = true;
        last_cmd_time_ = ros::Time::now();
    }

    void EstimateParameters::get_parameters(double T, double dt, double high_hop) {
        if (T <= 0.0) {
            ROS_WARN("Invalid T=%f, fallback to 0.5", T);
            this->T = 0.4;
        } else {
            this->T = T;
        }

        if (dt <= 0.0) {
            ROS_WARN("Invalid dt=%f, fallback to 0.1", dt);
            this->dt = 0.1;
        } else {
            this->dt = dt;
        }

        this->high_hop = (high_hop < 0.0) ? 0.0 : high_hop;
    }

    void EstimateParameters::Odom_init() {
        odom_TF.header.stamp = _currentTime;
        odom_TF.header.frame_id = odom_frame_id_;
        odom_TF.child_frame_id = base_frame_id_;

        odom_TF.transform.translation.x = body_position_world[0];
        odom_TF.transform.translation.y = body_position_world[1];
        odom_TF.transform.translation.z = body_position_world[2];
        odom_TF.transform.rotation.x = quaternion.x();
        odom_TF.transform.rotation.y = quaternion.y();
        odom_TF.transform.rotation.z = quaternion.z();
        odom_TF.transform.rotation.w = quaternion.w();

        odom_broadcaster.sendTransform(odom_TF);
    
        odom.header.stamp = _currentTime;
        odom.header.frame_id = odom_frame_id_;

        odom.pose.pose.position.x = body_position_world[0];
        odom.pose.pose.position.y = body_position_world[1];
        odom.pose.pose.position.z = body_position_world[2];
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        odom.pose.pose.orientation.w = quaternion.w();
    
        odom.child_frame_id = base_frame_id_;

        Eigen::Vector3d lin, ang;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            lin = body_velocity_world;
            // 使用传感器角速度优先反映真实旋转，否则使用指令值
            ang = imu_received_ ? imu_ang_vel_ : body_angular_velocity;
        }

        odom.twist.twist.linear.x = lin[0];
        odom.twist.twist.linear.y = lin[1];
        odom.twist.twist.linear.z = lin[2];
        odom.twist.twist.angular.x = ang[0];
        odom.twist.twist.angular.y = ang[1];
        odom.twist.twist.angular.z = ang[2];

        odom_pub.publish(odom);
    }

    void EstimateParameters::GetWorldPosition() {
        const double delta = (loop_dt_ > 0.0) ? loop_dt_ : T;
        Eigen::Vector3d vel;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            vel = body_velocity_world;
        }
        body_position_world[0] += (vel[0]*cos(eulerAngle[2])-vel[1]*sin(eulerAngle[2]))*delta;
        body_position_world[1] += (vel[0]*sin(eulerAngle[2])+vel[1]*cos(eulerAngle[2]))*delta;

        FL_velocity_world = vel;
        RL_velocity_world = vel;
        FR_velocity_world = vel;
        RR_velocity_world = vel;
    }

    void EstimateParameters::Get_R_world_to_body() {
        const double delta = (loop_dt_ > 0.0) ? loop_dt_ : T;
        Eigen::Vector3d ang_vel;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ang_vel =0.7 * body_angular_velocity;
        }

        if (use_imu_orientation_ && imu_received_) {
            // 姿态由IMU提供，保持指令角速度用于步态/运动控制
            quaternion = imu_quat_.normalized();
            R_world_to_body = quaternion.toRotationMatrix();
            eulerAngle = R_world_to_body.eulerAngles(0,1,2);
        } else {
            eulerAngle[0] += ang_vel[0]*delta;
            eulerAngle[1] += ang_vel[1]*delta;
            eulerAngle[2] += ang_vel[2]*delta;

            Eigen::AngleAxisd rollAngle(eulerAngle(0), Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(eulerAngle(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(eulerAngle(2), Eigen::Vector3d::UnitZ());

            quaternion = (yawAngle*pitchAngle*rollAngle).normalized();
            R_world_to_body = quaternion.toRotationMatrix();
        }
    }


    int EstimateParameters::GetFeetRotatePos() {
        if (T <= 0.0 || dt <= 0.0) {
            ROS_WARN_THROTTLE(2.0, "Invalid gait params T=%f dt=%f", T, dt);
            return 0;
        }

        Eigen::Vector3d ang_vel;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            ang_vel = body_angular_velocity;
        }
        if (ang_vel.isZero(1e-6) ) {
            gait_time_rotate_ = 0.0;
            leg_sign.clear();
            return 0;
        }

        const double step = (loop_dt_ > 0.0) ? loop_dt_ : dt;
        if (gait_time_rotate_ >= (T+0.05)) {
                gait_time_rotate_ = 0;
        }
        const double phase = std::fmod(gait_time_rotate_, T+0.1);
        // const double phase = gait_time_rotate_;

        const double yaw_step = ang_vel[2] * T;

        Eigen::AngleAxisd rot_vec(yaw_step, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d R = rot_vec.toRotationMatrix();
        Eigen::Matrix<double,3,4> current_position = R * init_position - init_position;
        // ROS_INFO_STREAM("phase:" << phase << " Current rotated foot positions:\n" << current_position);

        if (phase <= (T/2.0)+0.05) {
            FL_position_body = 2*phase/T*current_position.block(0,0,3,1);
            RR_position_body = 2*phase/T*current_position.block(0,3,3,1);
            FL_position_body[2] = high_hop*sin(2*phase*M_PI/T);
            RR_position_body[2] = high_hop*sin(2*phase*M_PI/T);
            RL_position_body.setZero();
            FR_position_body.setZero();
            leg_sign = {0,3};
  
        } else {
            const double sub_phase = phase - T/2.0;
            FL_position_body.setZero();
            RR_position_body.setZero();
            RL_position_body = 2*sub_phase/T*current_position.block(0,1,3,1);
            FR_position_body = 2*sub_phase/T*current_position.block(0,2,3,1);
            RL_position_body[2] = high_hop*sin(2*sub_phase*M_PI/T);
            FR_position_body[2] = high_hop*sin(2*sub_phase*M_PI/T);
            leg_sign = {1,2};

        }

        All_Feet_position.block(0,0,3,1)=FL_position_body;
        All_Feet_position.block(0,1,3,1)=RL_position_body;
        All_Feet_position.block(0,2,3,1)=FR_position_body;
        All_Feet_position.block(0,3,3,1)=RR_position_body;
        move.commandSend(All_Feet_position,{0,1,2,3});

        gait_time_rotate_ += step;
        
        return 1;     
    }

    int EstimateParameters::GetFeetMovePos() {
        if (T <= 0.0 || dt <= 0.0) {
            ROS_WARN_THROTTLE(2.0, "Invalid gait params T=%f dt=%f", T, dt);
            return 0;
        }

        Eigen::Vector3d vel;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            vel = body_velocity_world;
        }
        if (vel.isZero(1e-6)) {
            gait_time_move_ = 0.0;
            leg_sign.clear();
            return 0;
        }

        FL_velocity_world = vel;
        RL_velocity_world = vel;
        FR_velocity_world = vel;
        RR_velocity_world = vel;

        const double step = (loop_dt_ > 0.0) ? loop_dt_ : dt;
        if (gait_time_move_ >= (T-0.05)) {
                gait_time_move_ = 0;
        }
        
        leg_sign = {0,1,2,3};
        
        if (gait_time_move_ < (T/2.0-0.05)) {
            FL_position_body = gait_time_move_*FL_velocity_world*sin(gait_time_move_ * M_PI / (T/2.0))*compensation_velocity;
            RR_position_body = gait_time_move_*RR_velocity_world*sin(gait_time_move_ * M_PI / (T/2.0));
            double height = high_hop * cos(gait_time_move_ * M_PI / (T/2.0));
            FL_position_body[2] = height;
            RR_position_body[2] = height;
            RL_position_body = -gait_time_move_*RL_velocity_world*sin(gait_time_move_ * M_PI / (T/2.0))*compensation_velocity;
            FR_position_body = -gait_time_move_*FR_velocity_world*sin(gait_time_move_ * M_PI / (T/2.0));
            leg_sign = {0, 1, 2, 3};
        } else {
            RL_position_body = (gait_time_move_ - T/2.0)*RL_velocity_world*sin((gait_time_move_ - T/2.0) * M_PI / (T/2.0))*compensation_velocity;
            FR_position_body = (gait_time_move_ - T/2.0)*FR_velocity_world*sin((gait_time_move_ - T/2.0) * M_PI / (T/2.0));
            double height = high_hop * cos((gait_time_move_ - T/2.0) * M_PI / (T/2.0));
            RL_position_body[2] = height;
            FR_position_body[2] = height;
            FL_position_body = -(gait_time_move_ - T/2.0)*FL_velocity_world*sin((gait_time_move_ - T/2.0) * M_PI / (T/2.0))*compensation_velocity;
            RR_position_body = -(gait_time_move_ - T/2.0)*RR_velocity_world*sin((gait_time_move_ - T/2.0) * M_PI / (T/2.0));
            leg_sign = {0, 1, 2, 3};
        }


        All_Feet_position.block(0,0,3,1)=FL_position_body;
        All_Feet_position.block(0,1,3,1)=RL_position_body;
        All_Feet_position.block(0,2,3,1)=FR_position_body;
        All_Feet_position.block(0,3,3,1)=RR_position_body;
        move.commandSend(All_Feet_position, leg_sign);
        // ROS_INFO_STREAM("All_Feet_Position:\n" << All_Feet_position);

        gait_time_move_ += step;
        return 1;
    }

    void EstimateParameters::disableCanonicalMode() {
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
    }

    void EstimateParameters::keyboardListenerThread() {
        ROS_INFO("Keyboard listener started. Use WASD to move, JK to rotate, Q to quit.");
    
        while (ros::ok()) {
            fd_set set;
            struct timeval timeout;
            FD_ZERO(&set);
            FD_SET(STDIN_FILENO, &set);
            timeout.tv_sec = 0;
            timeout.tv_usec = 10000;
        
            int result = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
        
            if (result > 0) {
                char c;
                if (read(STDIN_FILENO, &c, 1) > 0) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (c == 'z') {
                        ROS_INFO("Quitting...");
                        ros::shutdown();
                        return;
                    }

                    if (c != last_key_) {
                        last_key_ = c;
                        switch (c) {
                            case 'w': body_velocity_world <<  0.2,  0.0, 0.0;  
                                      body_velocity_world *= lin_gain_;
                                      body_angular_velocity.setZero();
                                      break;
                            case 's': body_velocity_world << -0.2,  0.0, 0.0;  
                                      body_velocity_world *= lin_gain_;
                                      body_angular_velocity.setZero();
                                      break;
                            case 'a': body_velocity_world <<  0.0,  0.2, 0.0;  
                                      body_velocity_world *= lin_gain_;
                                      body_angular_velocity.setZero();
                                      break;
                            case 'd': body_velocity_world <<  0.0, -0.2, 0.0;  
                                      body_velocity_world *= lin_gain_;
                                      body_angular_velocity.setZero();
                                      break;
                            case 'q': body_angular_velocity << 0.0, 0.0, 0.2; 
                                      body_velocity_world.setZero();
                                      break;
                            case 'e': body_angular_velocity << 0.0, 0.0,-0.2; 
                                      body_velocity_world.setZero();
                                      break;
                            case 'x': body_velocity_world.setZero();
                                      body_angular_velocity.setZero();
                                      break;
                            default:  break;
                        }
                    }
                    keyboard_active_ = true;
                    last_key_time_ = ros::Time::now();
                    last_cmd_time_ = last_key_time_;
                }
            }
        
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (keyboard_active_ && last_key_ != '\0') {
                    double time_since_last_key = (ros::Time::now() - last_key_time_).toSec();
                    if (time_since_last_key > 0.15) {
                        last_key_ = '\0';
                        keyboard_active_ = false;
                        body_velocity_world.setZero();
                        body_angular_velocity.setZero();
                    }
                }
            }
        }
    }

    void EstimateParameters::mainloop() {
        while (ros::ok() && ros::Time::now().isZero()) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        // ros::Duration(0.5).sleep();

        _lastTime = ros::Time::now();
        _currentTime = _lastTime;
        Odom_init();

        if (T <= 0.0 || dt <= 0.0) {
            ROS_WARN("Invalid initial gait params T=%f dt=%f, resetting to defaults", T, dt);
            T = 0.4;
            dt = 0.05;
        }

        target_hz_ = std::clamp(1.0 / dt, 1.0, 100.0);
        const double period = 1.0 / target_hz_;
        ROS_INFO("Navigation loop freq set to %.1f Hz (dt=%.3f)", target_hz_, period);

        update_timer_ = nh.createTimer(ros::Duration(period), &EstimateParameters::UpdateTimerCb, this);
        started_ = true;
    }

    void EstimateParameters::UpdateTimerCb(const ros::TimerEvent& event) {
        if (!started_) return;

        _currentTime = ros::Time::now();
        // ROS_INFO_THROTTLE(1.0, "Current Time: %f", _currentTime.toSec());

        double computed_dt = (_lastTime.isZero()) ? (1.0/target_hz_) : (_currentTime - _lastTime).toSec();
        if (computed_dt <= 0.0 || computed_dt > 1.0) {
            computed_dt = 1.0/target_hz_;
        }
        loop_dt_ = computed_dt;
        _lastTime = _currentTime;
        // ROS_INFO("Loop dt: %.4f s", loop_dt_);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (keyboard_active_) {
                double dt_key = (_currentTime - last_key_time_).toSec();
                if (dt_key > key_timeout_) {
                    keyboard_active_ = false;
                    body_velocity_world.setZero();
                    body_angular_velocity.setZero();
                }
            } else {
                double dt_cmd = (_currentTime - last_cmd_time_).toSec();
                if (dt_cmd > cmd_timeout_) {
                    body_velocity_world.setZero();
                    body_angular_velocity.setZero();
                }
            }
        }

        GetWorldPosition();
        Get_R_world_to_body();
        GetFeetRotatePos();
        GetFeetMovePos();
        Odom_init();

        if(body_angular_velocity.isZero() && body_velocity_world.isZero()){
            move.stand();
        }
    }